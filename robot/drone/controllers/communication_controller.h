#pragma once

#include <memory>
#include "cmsis_os.h"
#include "bsp_usb.h"
#include "minipc_protocol.h"
#include "system_state_manager.h"
#include "vehicle_controller.h"
#include "imu_controller.h"
#include "self_test_controller.h"
#include "../common/task_config.h"

//==================================================================================================
// Communication Controller Class
//==================================================================================================

class CommunicationController {
public:
    static constexpr uint32_t TASK_DELAY_MS = COMMUNICATION_TASK_DELAY_MS;
    static constexpr uint32_t USB_RX_SIGNAL = (1 << 0);

    class CustomCommUSB : public bsp::VirtualUSB {
    protected:
        void RxCompleteCallback() override final {
            if (task_handle_) {
                osThreadFlagsSet(task_handle_, USB_RX_SIGNAL);
            }
        }

    public:
        void SetTaskHandle(osThreadId_t handle) {
            task_handle_ = handle;
        }

    private:
        osThreadId_t task_handle_ = nullptr;
    };

    CommunicationController() = default;

    void Init(SystemStateManager* state_manager,
              VehicleController* vehicle_ctrl,
              IMUController* imu_ctrl,
              SelfTestController* self_test);

    void SetTaskHandle(osThreadId_t handle);

    void Execute();

private:
    static constexpr uint32_t PC_TIMEOUT_MS = 2000;
    static constexpr uint32_t IMU_SEND_PERIOD = 10;
    static constexpr uint32_t ODOM_SEND_PERIOD = 20;
    static constexpr uint32_t STATUS_SEND_PERIOD = 200;

    SystemStateManager* state_manager_ = nullptr;
    VehicleController* vehicle_ctrl_ = nullptr;
    IMUController* imu_ctrl_ = nullptr;
    SelfTestController* self_test_ = nullptr;

    std::unique_ptr<CustomCommUSB> comm_usb_;
    std::unique_ptr<communication::MinipcPort> minipc_session_;

    uint32_t last_pc_data_time_ = 0;
    uint32_t last_imu_send_ = 0;
    uint32_t last_odom_send_ = 0;
    uint32_t last_status_send_ = 0;

    // Selfcheck statistics
    struct {
        uint32_t tx_count = 0;
        uint32_t rx_count = 0;
        uint32_t error_count = 0;
        uint32_t last_ping_time = 0;
        uint8_t last_ping_seq = 0;
    } selfcheck_stats_;

    void HandleMotionCommand(const communication::status_data_t* status);
    void HandleSelfcheckCommand(const communication::status_data_t* status);
    uint8_t GetSelfcheckStatusValue(uint8_t query_type);
    uint8_t ExecuteSelfcheckReset(uint8_t reset_type);
    void SendSelfcheckResponse(const communication::selfcheck_data_t* response_data);
    void SendIMUData();
    void SendOdometryData();
    void SendSystemStatus();
};