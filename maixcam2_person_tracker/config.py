"""Central configuration for the Stage 1 person-tracking build."""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class ModelConfig:
    path: str = "/root/models/yolo11n.mud"
    target_class_id: int = 0
    target_label: str = "person"
    confidence_threshold: float = 0.45
    iou_threshold: float = 0.45
    dual_buffer: bool = True


@dataclass(frozen=True)
class CameraConfig:
    buffer_count: int = 1
    horizontal_fov_deg: float = 87.0
    vertical_fov_deg: float = 49.0
    pan_pixel_sign: float = 1.0
    tilt_pixel_sign: float = -1.0


@dataclass(frozen=True)
class SelectionConfig:
    confirmation_frames: int = 3
    reacquire_gate_deg: float = 18.0
    center_weight: float = 0.30
    confidence_weight: float = 0.45
    area_weight: float = 0.25


@dataclass(frozen=True)
class FilterConfig:
    angular_acceleration_noise: float = 80.0
    measurement_noise_deg: float = 1.5
    initial_position_variance: float = 16.0
    initial_rate_variance: float = 400.0
    maximum_prediction_step_s: float = 0.10


@dataclass(frozen=True)
class StateMachineConfig:
    boot_feedback_timeout_ms: int = 2500
    coast_timeout_ms: int = 450
    search_timeout_ms: int = 2500
    maximum_coast_sigma_deg: float = 12.0
    feedback_stale_timeout_ms: int = 500


@dataclass(frozen=True)
class AxisConfig:
    motor_id: int
    motor_sign: float
    zero_offset_deg: float
    minimum_deg: float
    maximum_deg: float
    maximum_rate_deg_s: float
    maximum_acceleration_deg_s2: float


@dataclass(frozen=True)
class GimbalConfig:
    # Keep False until wiring, axis signs, zero offsets, and soft limits are verified.
    enabled: bool = False
    uart_device: str = "/dev/ttyS4"
    uart_tx_pin: str = "A21"
    uart_rx_pin: str = "A22"
    baud_rate: int = 115200
    inter_frame_delay_ms: int = 2
    command_period_ms: int = 20
    feedback_request_period_ms: int = 20
    motor_speed_limit_rpm: int = 20
    motor_acceleration_rpm_s: int = 100
    prediction_lookahead_s: float = 0.08
    disable_on_fault: bool = True
    disable_on_normal_exit: bool = False
    pan: AxisConfig = field(
        default_factory=lambda: AxisConfig(
            motor_id=1,
            motor_sign=1.0,
            zero_offset_deg=0.0,
            minimum_deg=-150.0,
            maximum_deg=150.0,
            maximum_rate_deg_s=60.0,
            maximum_acceleration_deg_s2=160.0,
        )
    )
    tilt: AxisConfig = field(
        default_factory=lambda: AxisConfig(
            motor_id=2,
            motor_sign=1.0,
            zero_offset_deg=0.0,
            minimum_deg=-50.0,
            maximum_deg=50.0,
            maximum_rate_deg_s=40.0,
            maximum_acceleration_deg_s2=120.0,
        )
    )


@dataclass(frozen=True)
class SearchConfig:
    initial_pan_amplitude_deg: float = 3.0
    initial_tilt_amplitude_deg: float = 2.0
    pan_growth_deg_s: float = 7.0
    tilt_growth_deg_s: float = 4.0
    maximum_pan_amplitude_deg: float = 35.0
    maximum_tilt_amplitude_deg: float = 18.0
    pan_frequency_hz: float = 0.30
    tilt_frequency_ratio: float = 0.61


@dataclass(frozen=True)
class UiConfig:
    enabled: bool = False
    overlay_scale: int = 1


@dataclass(frozen=True)
class TelemetryConfig:
    enabled: bool = True
    csv_path: str = "/root/logs/person_tracker.csv"
    sample_period_ms: int = 100
    queue_capacity: int = 256


@dataclass(frozen=True)
class WorkbenchConfig:
    # Set this to the Mac's USB-network URL ending in /api/telemetry.
    endpoint: str = ""
    publish_period_ms: int = 250


@dataclass(frozen=True)
class AppConfig:
    model: ModelConfig = field(default_factory=ModelConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    selection: SelectionConfig = field(default_factory=SelectionConfig)
    filter: FilterConfig = field(default_factory=FilterConfig)
    state_machine: StateMachineConfig = field(default_factory=StateMachineConfig)
    gimbal: GimbalConfig = field(default_factory=GimbalConfig)
    search: SearchConfig = field(default_factory=SearchConfig)
    ui: UiConfig = field(default_factory=UiConfig)
    telemetry: TelemetryConfig = field(default_factory=TelemetryConfig)
    workbench: WorkbenchConfig = field(default_factory=WorkbenchConfig)


CONFIG = AppConfig()
