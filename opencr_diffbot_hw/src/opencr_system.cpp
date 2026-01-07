#include "opencr_diffbot_hw/opencr_system.hpp"
#include <stdexcept>
#include "pluginlib/class_list_macros.hpp"

// #include "rclcpp/rclcpp.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>


namespace opencr_diffbot_hw
{
    hardware_interface::CallbackReturn OpenCRSystem::on_init(const hardware_interface::HardwareInfo & info)
    {
        // 반드시 부모 on_init을 먼저 호출 (info_ 세팅 포함)
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // ---- 1) ros2_control에서 넘어오는 파라미터 읽기 ----
        // (주의) URDF 쪽에서 키 이름을 어떻게 쓰는지에 따라 달라질 수 있어 100% 단정은 어렵습니다.
        // 하지만 ros2_control 관례상 hardware_parameters로 넘어옵니다.
        try {
            if (info_.hardware_parameters.find("device") != info_.hardware_parameters.end()) {
            device_ = info_.hardware_parameters.at("device");
            }
            if (info_.hardware_parameters.find("baud") != info_.hardware_parameters.end()) {
            baud_ = std::stoi(info_.hardware_parameters.at("baud"));
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR(rclcpp::get_logger("OpenCRSystem"), "Parameter parse error: %s", e.what());

            return hardware_interface::CallbackReturn::ERROR;
        }

        // ---- 2) 조인트 개수/이름 검증 ----
        // diff_drive_controller의 기본 형태는 보통 바퀴 2개(좌/우) 조인트를 가정합니다.
        if (info_.joints.size() != 2) {
            RCLCPP_ERROR(
            rclcpp::get_logger("OpenCRSystem"),
            "Expected 2 joints, got %zu", info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        left_joint_name_  = info_.joints[0].name;
        right_joint_name_ = info_.joints[1].name;

        // ---- 3) 내부 버퍼 초기화 ----
        pos_.assign(2, 0.0);
        vel_.assign(2, 0.0);
        cmd_vel_.assign(2, 0.0);

        RCLCPP_INFO(
            rclcpp::get_logger("OpenCRSystem"),
            "Initialized. device=%s baud=%d joints=[%s, %s]",
            device_.c_str(), baud_, left_joint_name_.c_str(), right_joint_name_.c_str());

        return hardware_interface::CallbackReturn::SUCCESS;
    }



    hardware_interface::CallbackReturn OpenCRSystem::on_configure(const rclcpp_lifecycle::State &)
    {
        // 시리얼 오픈
        serial_ok_ = open_serial_();
        if (!serial_ok_) {
            RCLCPP_ERROR(rclcpp::get_logger("OpenCRSystem"), "Failed to open serial: %s", device_.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }


        rx_buf_.clear();

        RCLCPP_INFO(
            rclcpp::get_logger("OpenCRSystem"),
            "Configured. Serial opened: %s @ %d", device_.c_str(), baud_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }



    hardware_interface::CallbackReturn OpenCRSystem::on_activate(const rclcpp_lifecycle::State &)
    {
        // Step 4에서는 안전을 위해 command를 0으로 시작한다고 가정
        cmd_vel_[0] = 0.0;
        cmd_vel_[1] = 0.0;

        RCLCPP_INFO(rclcpp::get_logger("OpenCRSystem"), "Activated (stub).");
        return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn OpenCRSystem::on_deactivate(const rclcpp_lifecycle::State &)
        {
        // 가능한 한 안전하게 정지 명령을 한번 보내고 닫기
        (void)write_cmd_(0.0, 0.0);
        close_serial_();

        RCLCPP_INFO(rclcpp::get_logger("OpenCRSystem"), "Deactivated. Serial closed.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }


    std::vector<hardware_interface::StateInterface> OpenCRSystem::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // Joint 0: left
        state_interfaces.emplace_back(
            left_joint_name_, hardware_interface::HW_IF_POSITION, &pos_[0]);
        state_interfaces.emplace_back(
            left_joint_name_, hardware_interface::HW_IF_VELOCITY, &vel_[0]);

        // Joint 1: right
        state_interfaces.emplace_back(
            right_joint_name_, hardware_interface::HW_IF_POSITION, &pos_[1]);
        state_interfaces.emplace_back(
            right_joint_name_, hardware_interface::HW_IF_VELOCITY, &vel_[1]);

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> OpenCRSystem::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // diff_drive_controller는 보통 바퀴에 "velocity command"를 씁니다.
        command_interfaces.emplace_back(
            left_joint_name_, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[0]);
        command_interfaces.emplace_back(
            right_joint_name_, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[1]);

        return command_interfaces;
    }

    hardware_interface::return_type OpenCRSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (!serial_ok_ || fd_ < 0) {
            return hardware_interface::return_type::ERROR;
        }

        // 한 루프에서 여러 줄이 쌓일 수 있으니, 가능한 만큼 처리
        // (너무 오래 돌면 controller loop를 방해할 수 있으니 제한을 둠)
        for (int i = 0; i < 10; ++i) {
            std::string line;
            if (!read_line_(line)) {
            break;  // 더 이상 완전한 한 줄이 없음
            }
            // line에는 '\n' 없이 들어오도록 구현할 예정
            (void)parse_state_line_(line);
        }

        return hardware_interface::return_type::OK;
    }


    hardware_interface::return_type OpenCRSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        if (!serial_ok_ || fd_ < 0) {
            return hardware_interface::return_type::ERROR;
        }

        // cmd_vel_는 rad/s 단위 (diff_drive_controller의 wheel velocity interface가 rad/s라고 가정)
        // 일반적으로 맞지만, URDF/컨트롤러 설정에 따라 해석이 달라질 가능성은 있습니다(100% 단정 불가).
        const double w1 = cmd_vel_[0];
        const double w2 = cmd_vel_[1];

        if (!write_cmd_(w1, w2)) {
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }



    ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////
    // below is the termios helper functions..
    static speed_t baud_to_termios(int baud)
    {
        switch (baud) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            default:
            // 교육용: 우선 115200으로 폴백
            return B115200;
        }
    }

    bool OpenCRSystem::open_serial_()
    {
        close_serial_();

        fd_ = ::open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        
        if (fd_ < 0) {
                RCLCPP_ERROR(
                rclcpp::get_logger("OpenCRSystem"),
                "open(%s) failed: %s", device_.c_str(), std::strerror(errno));
                return false;
        }

        termios tio{};
        if (tcgetattr(fd_, &tio) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("OpenCRSystem"), "tcgetattr failed: %s", std::strerror(errno));
            close_serial_();
            return false;
        }

        cfmakeraw(&tio);

        // baud
        const speed_t spd = baud_to_termios(baud_);
        cfsetispeed(&tio, spd);
        cfsetospeed(&tio, spd);

        // 8N1
        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~PARENB;
        tio.c_cflag &= ~CSTOPB;
        tio.c_cflag &= ~CSIZE;
        tio.c_cflag |= CS8;

        // non-blocking read에서도 동작하도록
        tio.c_cc[VMIN]  = 0;
        tio.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("OpenCRSystem"), "tcsetattr failed: %s", std::strerror(errno));
            close_serial_();
            return false;
        }

        tcflush(fd_, TCIOFLUSH);
        return true;
    }

    void OpenCRSystem::close_serial_()
    {
            if (fd_ >= 0) {
                ::close(fd_);
                fd_ = -1;
            }
            serial_ok_ = false;
    }

    bool OpenCRSystem::read_line_(std::string & line_out)
    {
        line_out.clear();

        // 먼저 기존 버퍼에 '\n'이 있으면 한 줄 꺼내기
        auto pos_nl = rx_buf_.find('\n');
        if (pos_nl != std::string::npos) {
            line_out = rx_buf_.substr(0, pos_nl);
            rx_buf_.erase(0, pos_nl + 1);
            // '\r' 제거(윈도우 스타일 대비)
            if (!line_out.empty() && line_out.back() == '\r') line_out.pop_back();
            return true;
        }

        // 없으면 fd에서 읽어와서 버퍼에 추가
        char tmp[256];
        const ssize_t n = ::read(fd_, tmp, sizeof(tmp));
        if (n > 0) {
            rx_buf_.append(tmp, tmp + n);
            // 다시 '\n' 찾기
            pos_nl = rx_buf_.find('\n');
            if (pos_nl != std::string::npos) {
            line_out = rx_buf_.substr(0, pos_nl);
            rx_buf_.erase(0, pos_nl + 1);
            if (!line_out.empty() && line_out.back() == '\r') line_out.pop_back();
            return true;
            }
        } else if (n < 0) {
            // EAGAIN/EWOULDBLOCK은 “읽을 게 없음”이라 정상
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_WARN(
                rclcpp::get_logger("OpenCRSystem"),
                "serial read error: %s", std::strerror(errno));
            }
        }

        return false;
    }


    bool OpenCRSystem::parse_state_line_(const std::string & line)
    {
            // OpenCR 출력 포맷(당신 코드 기준):
            // S ms w1 w2 pos1 pos2 ticks1 ticks2 pwm1 pwm2
            if (line.empty() || line[0] != 'S') {
                return false;
        }
        

        // 가장 필요한 값: w1, w2, pos1, pos2
        // ticks/pwm은 일단 사용 안 함(필요하면 나중에 확장)
        long ms = 0;
        double w1 = 0.0, w2 = 0.0, p1 = 0.0, p2 = 0.0;
        long t1 = 0, t2 = 0;
        int pwm1 = 0, pwm2 = 0;

        const int ret = std::sscanf(
            line.c_str(),
            "S %ld %lf %lf %lf %lf %ld %ld %d %d",
            &ms, &w1, &w2, &p1, &p2, &t1, &t2, &pwm1, &pwm2);

        if (ret < 5) {
            // 최소 w1,w2,pos1,pos2까지 못 읽으면 실패
            return false;
        }

        // 여기서 pos_/vel_ 업데이트
        // 주의: 이 값들이 "조인트(바퀴) 회전"의 rad, rad/s로 들어온다는 전제입니다.
        // 당신 OpenCR 펌웨어는 pos(rad), w(rad/s)로 출력하므로 일치합니다.
        vel_[0] = w1;
        vel_[1] = w2;

        pos_[0] = p1;
        pos_[1] = p2;

        return true;
    }

    bool OpenCRSystem::write_cmd_(double w1_radps, double w2_radps)
    {
        // OpenCR 입력 포맷:
        // V w1_rad_s w2_rad_s
        // 여기서는 줄 끝에 '\n' 필수
        char buf[96];
        const int n = std::snprintf(buf, sizeof(buf), "V %.6f %.6f\n", w1_radps, w2_radps);
        if (n <= 0) return false;

        const ssize_t wr = ::write(fd_, buf, static_cast<size_t>(n));
        if (wr < 0) {
            // EAGAIN은 드물지만, 비차단이면 가능. 여기서는 경고만.
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_WARN(
                rclcpp::get_logger("OpenCRSystem"),
                "serial write error: %s", std::strerror(errno));
            return false;
            }
        }

        return true;
    }
    


}  // namespace opencr_diffbot_hw

// pluginlib 등록 매크로: XML의 type="opencr_diffbot_hw::OpenCRSystem"와 연결됨
PLUGINLIB_EXPORT_CLASS(opencr_diffbot_hw::OpenCRSystem, hardware_interface::SystemInterface)
