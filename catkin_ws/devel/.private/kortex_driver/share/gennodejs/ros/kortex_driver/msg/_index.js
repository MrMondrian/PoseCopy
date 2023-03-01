
"use strict";

let KortexError = require('./KortexError.js');
let ApiOptions = require('./ApiOptions.js');
let SubErrorCodes = require('./SubErrorCodes.js');
let ErrorCodes = require('./ErrorCodes.js');
let CustomDataIndex = require('./CustomDataIndex.js');
let CommandMode = require('./CommandMode.js');
let StepResponse = require('./StepResponse.js');
let ActuatorConfig_ControlMode = require('./ActuatorConfig_ControlMode.js');
let ActuatorConfig_ControlModeInformation = require('./ActuatorConfig_ControlModeInformation.js');
let AxisPosition = require('./AxisPosition.js');
let RampResponse = require('./RampResponse.js');
let ControlLoop = require('./ControlLoop.js');
let ActuatorConfig_ServiceVersion = require('./ActuatorConfig_ServiceVersion.js');
let ActuatorConfig_SafetyLimitType = require('./ActuatorConfig_SafetyLimitType.js');
let FrequencyResponse = require('./FrequencyResponse.js');
let CommandModeInformation = require('./CommandModeInformation.js');
let CoggingFeedforwardModeInformation = require('./CoggingFeedforwardModeInformation.js');
let LoopSelection = require('./LoopSelection.js');
let ControlLoopParameters = require('./ControlLoopParameters.js');
let Servoing = require('./Servoing.js');
let TorqueCalibration = require('./TorqueCalibration.js');
let SafetyIdentifierBankA = require('./SafetyIdentifierBankA.js');
let CoggingFeedforwardMode = require('./CoggingFeedforwardMode.js');
let EncoderDerivativeParameters = require('./EncoderDerivativeParameters.js');
let TorqueOffset = require('./TorqueOffset.js');
let VectorDriveParameters = require('./VectorDriveParameters.js');
let PositionCommand = require('./PositionCommand.js');
let AxisOffsets = require('./AxisOffsets.js');
let ControlLoopSelection = require('./ControlLoopSelection.js');
let CustomDataSelection = require('./CustomDataSelection.js');
let ActuatorCyclic_ServiceVersion = require('./ActuatorCyclic_ServiceVersion.js');
let ActuatorCyclic_MessageId = require('./ActuatorCyclic_MessageId.js');
let ActuatorCyclic_Command = require('./ActuatorCyclic_Command.js');
let ActuatorCyclic_CustomData = require('./ActuatorCyclic_CustomData.js');
let CommandFlags = require('./CommandFlags.js');
let StatusFlags = require('./StatusFlags.js');
let ActuatorCyclic_Feedback = require('./ActuatorCyclic_Feedback.js');
let Base_ControlModeNotification = require('./Base_ControlModeNotification.js');
let SafetyEvent = require('./SafetyEvent.js');
let LimitationType = require('./LimitationType.js');
let ProtectionZoneList = require('./ProtectionZoneList.js');
let Base_ControlModeInformation = require('./Base_ControlModeInformation.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');
let JointsLimitationsList = require('./JointsLimitationsList.js');
let ControllerElementHandle = require('./ControllerElementHandle.js');
let ActionNotificationList = require('./ActionNotificationList.js');
let Orientation = require('./Orientation.js');
let WifiSecurityType = require('./WifiSecurityType.js');
let ArmStateInformation = require('./ArmStateInformation.js');
let Base_CapSenseConfig = require('./Base_CapSenseConfig.js');
let IPv4Configuration = require('./IPv4Configuration.js');
let ActivateMapHandle = require('./ActivateMapHandle.js');
let WrenchLimitation = require('./WrenchLimitation.js');
let SwitchControlMapping = require('./SwitchControlMapping.js');
let JointTorque = require('./JointTorque.js');
let ControllerConfigurationMode = require('./ControllerConfigurationMode.js');
let LedState = require('./LedState.js');
let ActionHandle = require('./ActionHandle.js');
let BackupEvent = require('./BackupEvent.js');
let RobotEvent = require('./RobotEvent.js');
let ProtectionZoneNotificationList = require('./ProtectionZoneNotificationList.js');
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let GpioEvent = require('./GpioEvent.js');
let JointNavigationDirection = require('./JointNavigationDirection.js');
let ProtectionZoneInformation = require('./ProtectionZoneInformation.js');
let ConstrainedOrientation = require('./ConstrainedOrientation.js');
let WrenchCommand = require('./WrenchCommand.js');
let NetworkHandle = require('./NetworkHandle.js');
let ControllerInputType = require('./ControllerInputType.js');
let Admittance = require('./Admittance.js');
let PreComputedJointTrajectory = require('./PreComputedJointTrajectory.js');
let Twist = require('./Twist.js');
let Gen3GpioPinId = require('./Gen3GpioPinId.js');
let ActionNotification = require('./ActionNotification.js');
let ActionType = require('./ActionType.js');
let Action_action_parameters = require('./Action_action_parameters.js');
let ControllerEvent = require('./ControllerEvent.js');
let BridgeType = require('./BridgeType.js');
let Base_ControlMode = require('./Base_ControlMode.js');
let GripperMode = require('./GripperMode.js');
let ChangeWrench = require('./ChangeWrench.js');
let Action = require('./Action.js');
let ControllerList = require('./ControllerList.js');
let TrajectoryErrorReport = require('./TrajectoryErrorReport.js');
let NetworkNotificationList = require('./NetworkNotificationList.js');
let SequenceList = require('./SequenceList.js');
let SequenceInfoNotification = require('./SequenceInfoNotification.js');
let ControllerEventType = require('./ControllerEventType.js');
let Base_Stop = require('./Base_Stop.js');
let SequenceTaskConfiguration = require('./SequenceTaskConfiguration.js');
let Base_RotationMatrix = require('./Base_RotationMatrix.js');
let MapEvent_events = require('./MapEvent_events.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let ShapeType = require('./ShapeType.js');
let JointAngles = require('./JointAngles.js');
let UserNotificationList = require('./UserNotificationList.js');
let TwistCommand = require('./TwistCommand.js');
let CartesianTrajectoryConstraint_type = require('./CartesianTrajectoryConstraint_type.js');
let NavigationDirection = require('./NavigationDirection.js');
let SequenceInformation = require('./SequenceInformation.js');
let TransformationMatrix = require('./TransformationMatrix.js');
let MapList = require('./MapList.js');
let CartesianLimitation = require('./CartesianLimitation.js');
let WaypointValidationReport = require('./WaypointValidationReport.js');
let IKData = require('./IKData.js');
let SequenceInfoNotificationList = require('./SequenceInfoNotificationList.js');
let IPv4Information = require('./IPv4Information.js');
let MapEvent = require('./MapEvent.js');
let NetworkType = require('./NetworkType.js');
let Sequence = require('./Sequence.js');
let WifiConfigurationList = require('./WifiConfigurationList.js');
let Faults = require('./Faults.js');
let Xbox360AnalogInputIdentifier = require('./Xbox360AnalogInputIdentifier.js');
let SequenceTask = require('./SequenceTask.js');
let GpioConfigurationList = require('./GpioConfigurationList.js');
let MappingHandle = require('./MappingHandle.js');
let Snapshot = require('./Snapshot.js');
let OperatingModeInformation = require('./OperatingModeInformation.js');
let ControllerElementEventType = require('./ControllerElementEventType.js');
let ProtectionZone = require('./ProtectionZone.js');
let ActionExecutionState = require('./ActionExecutionState.js');
let ConstrainedPosition = require('./ConstrainedPosition.js');
let SequenceTaskHandle = require('./SequenceTaskHandle.js');
let GripperCommand = require('./GripperCommand.js');
let ArmStateNotification = require('./ArmStateNotification.js');
let PreComputedJointTrajectoryElement = require('./PreComputedJointTrajectoryElement.js');
let UserProfileList = require('./UserProfileList.js');
let BridgeStatus = require('./BridgeStatus.js');
let SnapshotType = require('./SnapshotType.js');
let EventIdSequenceInfoNotification = require('./EventIdSequenceInfoNotification.js');
let ActionEvent = require('./ActionEvent.js');
let ServoingMode = require('./ServoingMode.js');
let SequenceHandle = require('./SequenceHandle.js');
let CartesianWaypoint = require('./CartesianWaypoint.js');
let AngularWaypoint = require('./AngularWaypoint.js');
let ConfigurationChangeNotificationList = require('./ConfigurationChangeNotificationList.js');
let ServoingModeNotification = require('./ServoingModeNotification.js');
let GripperRequest = require('./GripperRequest.js');
let RequestedActionType = require('./RequestedActionType.js');
let JointTorques = require('./JointTorques.js');
let ServoingModeInformation = require('./ServoingModeInformation.js');
let RobotEventNotification = require('./RobotEventNotification.js');
let Base_SafetyIdentifier = require('./Base_SafetyIdentifier.js');
let UserEvent = require('./UserEvent.js');
let ControllerNotification = require('./ControllerNotification.js');
let WrenchMode = require('./WrenchMode.js');
let FirmwareComponentVersion = require('./FirmwareComponentVersion.js');
let ConstrainedJointAngle = require('./ConstrainedJointAngle.js');
let ProtectionZoneNotification = require('./ProtectionZoneNotification.js');
let FullIPv4Configuration = require('./FullIPv4Configuration.js');
let ServoingModeNotificationList = require('./ServoingModeNotificationList.js');
let BridgeList = require('./BridgeList.js');
let BridgePortConfig = require('./BridgePortConfig.js');
let WifiInformation = require('./WifiInformation.js');
let ConfigurationChangeNotification_configuration_change = require('./ConfigurationChangeNotification_configuration_change.js');
let PasswordChange = require('./PasswordChange.js');
let GpioPinConfiguration = require('./GpioPinConfiguration.js');
let ControllerNotificationList = require('./ControllerNotificationList.js');
let JointLimitation = require('./JointLimitation.js');
let Wrench = require('./Wrench.js');
let FactoryNotification = require('./FactoryNotification.js');
let WristDigitalInputIdentifier = require('./WristDigitalInputIdentifier.js');
let SafetyNotificationList = require('./SafetyNotificationList.js');
let Waypoint_type_of_waypoint = require('./Waypoint_type_of_waypoint.js');
let WifiEncryptionType = require('./WifiEncryptionType.js');
let JointAngle = require('./JointAngle.js');
let OperatingMode = require('./OperatingMode.js');
let MapElement = require('./MapElement.js');
let SequenceTasksRange = require('./SequenceTasksRange.js');
let ProtectionZoneEvent = require('./ProtectionZoneEvent.js');
let SequenceTasksConfiguration = require('./SequenceTasksConfiguration.js');
let ControllerState = require('./ControllerState.js');
let Base_Position = require('./Base_Position.js');
let SystemTime = require('./SystemTime.js');
let TrajectoryContinuityMode = require('./TrajectoryContinuityMode.js');
let ActionList = require('./ActionList.js');
let ControlModeNotificationList = require('./ControlModeNotificationList.js');
let WifiConfiguration = require('./WifiConfiguration.js');
let TrajectoryErrorElement = require('./TrajectoryErrorElement.js');
let ControllerType = require('./ControllerType.js');
let Gripper = require('./Gripper.js');
let FullUserProfile = require('./FullUserProfile.js');
let ConstrainedPose = require('./ConstrainedPose.js');
let ChangeTwist = require('./ChangeTwist.js');
let ControllerConfigurationList = require('./ControllerConfigurationList.js');
let ControllerNotification_state = require('./ControllerNotification_state.js');
let Map = require('./Map.js');
let MapGroupList = require('./MapGroupList.js');
let ControllerBehavior = require('./ControllerBehavior.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');
let GpioCommand = require('./GpioCommand.js');
let TrajectoryErrorIdentifier = require('./TrajectoryErrorIdentifier.js');
let MappingInfoNotificationList = require('./MappingInfoNotificationList.js');
let Ssid = require('./Ssid.js');
let Waypoint = require('./Waypoint.js');
let MapHandle = require('./MapHandle.js');
let SignalQuality = require('./SignalQuality.js');
let NetworkNotification = require('./NetworkNotification.js');
let ConstrainedJointAngles = require('./ConstrainedJointAngles.js');
let BridgeConfig = require('./BridgeConfig.js');
let JointTrajectoryConstraintType = require('./JointTrajectoryConstraintType.js');
let Base_RotationMatrixRow = require('./Base_RotationMatrixRow.js');
let WifiInformationList = require('./WifiInformationList.js');
let SoundType = require('./SoundType.js');
let AdmittanceMode = require('./AdmittanceMode.js');
let GpioPinPropertyFlags = require('./GpioPinPropertyFlags.js');
let TransformationRow = require('./TransformationRow.js');
let Base_CapSenseMode = require('./Base_CapSenseMode.js');
let AppendActionInformation = require('./AppendActionInformation.js');
let BridgeResult = require('./BridgeResult.js');
let NetworkEvent = require('./NetworkEvent.js');
let ControllerConfiguration = require('./ControllerConfiguration.js');
let Mapping = require('./Mapping.js');
let TrajectoryErrorType = require('./TrajectoryErrorType.js');
let ControllerElementState = require('./ControllerElementState.js');
let ZoneShape = require('./ZoneShape.js');
let UserProfile = require('./UserProfile.js');
let CommunicationInterfaceConfiguration = require('./CommunicationInterfaceConfiguration.js');
let Base_ServiceVersion = require('./Base_ServiceVersion.js');
let CartesianSpeed = require('./CartesianSpeed.js');
let TrajectoryInfo = require('./TrajectoryInfo.js');
let Pose = require('./Pose.js');
let TwistLimitation = require('./TwistLimitation.js');
let Query = require('./Query.js');
let ConfigurationNotificationEvent = require('./ConfigurationNotificationEvent.js');
let WaypointList = require('./WaypointList.js');
let FirmwareBundleVersions = require('./FirmwareBundleVersions.js');
let RobotEventNotificationList = require('./RobotEventNotificationList.js');
let ControllerElementHandle_identifier = require('./ControllerElementHandle_identifier.js');
let FactoryEvent = require('./FactoryEvent.js');
let Timeout = require('./Timeout.js');
let UserList = require('./UserList.js');
let MapGroup = require('./MapGroup.js');
let ConfigurationChangeNotification = require('./ConfigurationChangeNotification.js');
let ControllerHandle = require('./ControllerHandle.js');
let TrajectoryInfoType = require('./TrajectoryInfoType.js');
let OperatingModeNotificationList = require('./OperatingModeNotificationList.js');
let CartesianLimitationList = require('./CartesianLimitationList.js');
let AdvancedSequenceHandle = require('./AdvancedSequenceHandle.js');
let Xbox360DigitalInputIdentifier = require('./Xbox360DigitalInputIdentifier.js');
let OperatingModeNotification = require('./OperatingModeNotification.js');
let KinematicTrajectoryConstraints = require('./KinematicTrajectoryConstraints.js');
let Delay = require('./Delay.js');
let MappingList = require('./MappingList.js');
let Finger = require('./Finger.js');
let UserNotification = require('./UserNotification.js');
let EmergencyStop = require('./EmergencyStop.js');
let Base_GpioConfiguration = require('./Base_GpioConfiguration.js');
let ChangeJointSpeeds = require('./ChangeJointSpeeds.js');
let MappingInfoNotification = require('./MappingInfoNotification.js');
let SequenceTasksPair = require('./SequenceTasksPair.js');
let GpioBehavior = require('./GpioBehavior.js');
let Point = require('./Point.js');
let JointSpeed = require('./JointSpeed.js');
let SequenceTasks = require('./SequenceTasks.js');
let ActuatorInformation = require('./ActuatorInformation.js');
let Base_JointSpeeds = require('./Base_JointSpeeds.js');
let GpioAction = require('./GpioAction.js');
let ActuatorCustomData = require('./ActuatorCustomData.js');
let BaseCyclic_ServiceVersion = require('./BaseCyclic_ServiceVersion.js');
let ActuatorFeedback = require('./ActuatorFeedback.js');
let BaseFeedback = require('./BaseFeedback.js');
let BaseCyclic_Command = require('./BaseCyclic_Command.js');
let BaseCyclic_Feedback = require('./BaseCyclic_Feedback.js');
let ActuatorCommand = require('./ActuatorCommand.js');
let BaseCyclic_CustomData = require('./BaseCyclic_CustomData.js');
let UserProfileHandle = require('./UserProfileHandle.js');
let SafetyStatusValue = require('./SafetyStatusValue.js');
let CountryCodeIdentifier = require('./CountryCodeIdentifier.js');
let NotificationOptions = require('./NotificationOptions.js');
let UARTDeviceIdentification = require('./UARTDeviceIdentification.js');
let NotificationHandle = require('./NotificationHandle.js');
let Connection = require('./Connection.js');
let NotificationType = require('./NotificationType.js');
let UARTSpeed = require('./UARTSpeed.js');
let SafetyNotification = require('./SafetyNotification.js');
let UARTParity = require('./UARTParity.js');
let DeviceTypes = require('./DeviceTypes.js');
let Permission = require('./Permission.js');
let ArmState = require('./ArmState.js');
let Empty = require('./Empty.js');
let Unit = require('./Unit.js');
let CountryCode = require('./CountryCode.js');
let SafetyHandle = require('./SafetyHandle.js');
let UARTWordLength = require('./UARTWordLength.js');
let Timestamp = require('./Timestamp.js');
let DeviceHandle = require('./DeviceHandle.js');
let UARTConfiguration = require('./UARTConfiguration.js');
let CartesianReferenceFrame = require('./CartesianReferenceFrame.js');
let UARTStopBits = require('./UARTStopBits.js');
let ControlConfig_ControlModeNotification = require('./ControlConfig_ControlModeNotification.js');
let JointAccelerationSoftLimits = require('./JointAccelerationSoftLimits.js');
let PayloadInformation = require('./PayloadInformation.js');
let ToolConfiguration = require('./ToolConfiguration.js');
let ControlConfigurationNotification = require('./ControlConfigurationNotification.js');
let ControlConfig_Position = require('./ControlConfig_Position.js');
let ControlConfig_ControlMode = require('./ControlConfig_ControlMode.js');
let LinearTwist = require('./LinearTwist.js');
let AngularTwist = require('./AngularTwist.js');
let ControlConfig_ServiceVersion = require('./ControlConfig_ServiceVersion.js');
let CartesianTransform = require('./CartesianTransform.js');
let ControlConfig_JointSpeeds = require('./ControlConfig_JointSpeeds.js');
let KinematicLimits = require('./KinematicLimits.js');
let TwistLinearSoftLimit = require('./TwistLinearSoftLimit.js');
let ControlConfigurationEvent = require('./ControlConfigurationEvent.js');
let DesiredSpeeds = require('./DesiredSpeeds.js');
let ControlConfig_ControlModeInformation = require('./ControlConfig_ControlModeInformation.js');
let CartesianReferenceFrameInfo = require('./CartesianReferenceFrameInfo.js');
let GravityVector = require('./GravityVector.js');
let KinematicLimitsList = require('./KinematicLimitsList.js');
let JointSpeedSoftLimits = require('./JointSpeedSoftLimits.js');
let TwistAngularSoftLimit = require('./TwistAngularSoftLimit.js');
let RebootRqst = require('./RebootRqst.js');
let CalibrationItem = require('./CalibrationItem.js');
let SafetyConfiguration = require('./SafetyConfiguration.js');
let SerialNumber = require('./SerialNumber.js');
let SafetyEnable = require('./SafetyEnable.js');
let DeviceConfig_CapSenseMode = require('./DeviceConfig_CapSenseMode.js');
let CalibrationElement = require('./CalibrationElement.js');
let DeviceType = require('./DeviceType.js');
let PowerOnSelfTestResult = require('./PowerOnSelfTestResult.js');
let PartNumber = require('./PartNumber.js');
let DeviceConfig_ServiceVersion = require('./DeviceConfig_ServiceVersion.js');
let CalibrationParameter = require('./CalibrationParameter.js');
let CalibrationParameter_value = require('./CalibrationParameter_value.js');
let IPv4Settings = require('./IPv4Settings.js');
let SafetyInformationList = require('./SafetyInformationList.js');
let DeviceConfig_SafetyLimitType = require('./DeviceConfig_SafetyLimitType.js');
let RunMode = require('./RunMode.js');
let CalibrationStatus = require('./CalibrationStatus.js');
let SafetyInformation = require('./SafetyInformation.js');
let Calibration = require('./Calibration.js');
let MACAddress = require('./MACAddress.js');
let RunModes = require('./RunModes.js');
let BootloaderVersion = require('./BootloaderVersion.js');
let DeviceConfig_CapSenseConfig = require('./DeviceConfig_CapSenseConfig.js');
let SafetyThreshold = require('./SafetyThreshold.js');
let PartNumberRevision = require('./PartNumberRevision.js');
let CapSenseRegister = require('./CapSenseRegister.js');
let ModelNumber = require('./ModelNumber.js');
let SafetyConfigurationList = require('./SafetyConfigurationList.js');
let SafetyStatus = require('./SafetyStatus.js');
let CalibrationResult = require('./CalibrationResult.js');
let FirmwareVersion = require('./FirmwareVersion.js');
let DeviceHandles = require('./DeviceHandles.js');
let DeviceManager_ServiceVersion = require('./DeviceManager_ServiceVersion.js');
let RobotiqGripperStatusFlags = require('./RobotiqGripperStatusFlags.js');
let GripperConfig_SafetyIdentifier = require('./GripperConfig_SafetyIdentifier.js');
let MotorFeedback = require('./MotorFeedback.js');
let CustomDataUnit = require('./CustomDataUnit.js');
let GripperCyclic_ServiceVersion = require('./GripperCyclic_ServiceVersion.js');
let GripperCyclic_Command = require('./GripperCyclic_Command.js');
let GripperCyclic_Feedback = require('./GripperCyclic_Feedback.js');
let GripperCyclic_MessageId = require('./GripperCyclic_MessageId.js');
let GripperCyclic_CustomData = require('./GripperCyclic_CustomData.js');
let MotorCommand = require('./MotorCommand.js');
let I2CConfiguration = require('./I2CConfiguration.js');
let InterconnectConfig_ServiceVersion = require('./InterconnectConfig_ServiceVersion.js');
let EthernetSpeed = require('./EthernetSpeed.js');
let EthernetDevice = require('./EthernetDevice.js');
let EthernetConfiguration = require('./EthernetConfiguration.js');
let I2CReadRegisterParameter = require('./I2CReadRegisterParameter.js');
let GPIOIdentification = require('./GPIOIdentification.js');
let InterconnectConfig_GPIOConfiguration = require('./InterconnectConfig_GPIOConfiguration.js');
let EthernetDuplex = require('./EthernetDuplex.js');
let I2CDeviceIdentification = require('./I2CDeviceIdentification.js');
let I2CReadParameter = require('./I2CReadParameter.js');
let GPIOValue = require('./GPIOValue.js');
let I2CRegisterAddressSize = require('./I2CRegisterAddressSize.js');
let InterconnectConfig_SafetyIdentifier = require('./InterconnectConfig_SafetyIdentifier.js');
let I2CDevice = require('./I2CDevice.js');
let UARTPortId = require('./UARTPortId.js');
let I2CMode = require('./I2CMode.js');
let GPIOMode = require('./GPIOMode.js');
let I2CWriteParameter = require('./I2CWriteParameter.js');
let I2CData = require('./I2CData.js');
let GPIOPull = require('./GPIOPull.js');
let GPIOIdentifier = require('./GPIOIdentifier.js');
let I2CDeviceAddressing = require('./I2CDeviceAddressing.js');
let GPIOState = require('./GPIOState.js');
let EthernetDeviceIdentification = require('./EthernetDeviceIdentification.js');
let I2CWriteRegisterParameter = require('./I2CWriteRegisterParameter.js');
let InterconnectCyclic_CustomData = require('./InterconnectCyclic_CustomData.js');
let InterconnectCyclic_Command = require('./InterconnectCyclic_Command.js');
let InterconnectCyclic_Feedback = require('./InterconnectCyclic_Feedback.js');
let InterconnectCyclic_Feedback_tool_feedback = require('./InterconnectCyclic_Feedback_tool_feedback.js');
let InterconnectCyclic_CustomData_tool_customData = require('./InterconnectCyclic_CustomData_tool_customData.js');
let InterconnectCyclic_ServiceVersion = require('./InterconnectCyclic_ServiceVersion.js');
let InterconnectCyclic_MessageId = require('./InterconnectCyclic_MessageId.js');
let InterconnectCyclic_Command_tool_command = require('./InterconnectCyclic_Command_tool_command.js');
let InterfaceModuleType = require('./InterfaceModuleType.js');
let BaseType = require('./BaseType.js');
let CompleteProductConfiguration = require('./CompleteProductConfiguration.js');
let ProductConfigurationEndEffectorType = require('./ProductConfigurationEndEffectorType.js');
let ModelId = require('./ModelId.js');
let EndEffectorType = require('./EndEffectorType.js');
let WristType = require('./WristType.js');
let VisionModuleType = require('./VisionModuleType.js');
let ArmLaterality = require('./ArmLaterality.js');
let Option = require('./Option.js');
let OptionInformation = require('./OptionInformation.js');
let OptionValue = require('./OptionValue.js');
let VisionEvent = require('./VisionEvent.js');
let FocusPoint = require('./FocusPoint.js');
let VisionConfig_ServiceVersion = require('./VisionConfig_ServiceVersion.js');
let FocusAction = require('./FocusAction.js');
let ManualFocus = require('./ManualFocus.js');
let VisionConfig_RotationMatrix = require('./VisionConfig_RotationMatrix.js');
let VisionNotification = require('./VisionNotification.js');
let Sensor = require('./Sensor.js');
let IntrinsicProfileIdentifier = require('./IntrinsicProfileIdentifier.js');
let DistortionCoefficients = require('./DistortionCoefficients.js');
let Resolution = require('./Resolution.js');
let OptionIdentifier = require('./OptionIdentifier.js');
let SensorFocusAction = require('./SensorFocusAction.js');
let BitRate = require('./BitRate.js');
let TranslationVector = require('./TranslationVector.js');
let SensorFocusAction_action_parameters = require('./SensorFocusAction_action_parameters.js');
let FrameRate = require('./FrameRate.js');
let SensorIdentifier = require('./SensorIdentifier.js');
let SensorSettings = require('./SensorSettings.js');
let VisionConfig_RotationMatrixRow = require('./VisionConfig_RotationMatrixRow.js');
let IntrinsicParameters = require('./IntrinsicParameters.js');
let ExtrinsicParameters = require('./ExtrinsicParameters.js');
let FollowCartesianTrajectoryActionFeedback = require('./FollowCartesianTrajectoryActionFeedback.js');
let FollowCartesianTrajectoryAction = require('./FollowCartesianTrajectoryAction.js');
let FollowCartesianTrajectoryActionResult = require('./FollowCartesianTrajectoryActionResult.js');
let FollowCartesianTrajectoryGoal = require('./FollowCartesianTrajectoryGoal.js');
let FollowCartesianTrajectoryResult = require('./FollowCartesianTrajectoryResult.js');
let FollowCartesianTrajectoryActionGoal = require('./FollowCartesianTrajectoryActionGoal.js');
let FollowCartesianTrajectoryFeedback = require('./FollowCartesianTrajectoryFeedback.js');

module.exports = {
  KortexError: KortexError,
  ApiOptions: ApiOptions,
  SubErrorCodes: SubErrorCodes,
  ErrorCodes: ErrorCodes,
  CustomDataIndex: CustomDataIndex,
  CommandMode: CommandMode,
  StepResponse: StepResponse,
  ActuatorConfig_ControlMode: ActuatorConfig_ControlMode,
  ActuatorConfig_ControlModeInformation: ActuatorConfig_ControlModeInformation,
  AxisPosition: AxisPosition,
  RampResponse: RampResponse,
  ControlLoop: ControlLoop,
  ActuatorConfig_ServiceVersion: ActuatorConfig_ServiceVersion,
  ActuatorConfig_SafetyLimitType: ActuatorConfig_SafetyLimitType,
  FrequencyResponse: FrequencyResponse,
  CommandModeInformation: CommandModeInformation,
  CoggingFeedforwardModeInformation: CoggingFeedforwardModeInformation,
  LoopSelection: LoopSelection,
  ControlLoopParameters: ControlLoopParameters,
  Servoing: Servoing,
  TorqueCalibration: TorqueCalibration,
  SafetyIdentifierBankA: SafetyIdentifierBankA,
  CoggingFeedforwardMode: CoggingFeedforwardMode,
  EncoderDerivativeParameters: EncoderDerivativeParameters,
  TorqueOffset: TorqueOffset,
  VectorDriveParameters: VectorDriveParameters,
  PositionCommand: PositionCommand,
  AxisOffsets: AxisOffsets,
  ControlLoopSelection: ControlLoopSelection,
  CustomDataSelection: CustomDataSelection,
  ActuatorCyclic_ServiceVersion: ActuatorCyclic_ServiceVersion,
  ActuatorCyclic_MessageId: ActuatorCyclic_MessageId,
  ActuatorCyclic_Command: ActuatorCyclic_Command,
  ActuatorCyclic_CustomData: ActuatorCyclic_CustomData,
  CommandFlags: CommandFlags,
  StatusFlags: StatusFlags,
  ActuatorCyclic_Feedback: ActuatorCyclic_Feedback,
  Base_ControlModeNotification: Base_ControlModeNotification,
  SafetyEvent: SafetyEvent,
  LimitationType: LimitationType,
  ProtectionZoneList: ProtectionZoneList,
  Base_ControlModeInformation: Base_ControlModeInformation,
  JointTrajectoryConstraint: JointTrajectoryConstraint,
  CartesianTrajectoryConstraint: CartesianTrajectoryConstraint,
  JointsLimitationsList: JointsLimitationsList,
  ControllerElementHandle: ControllerElementHandle,
  ActionNotificationList: ActionNotificationList,
  Orientation: Orientation,
  WifiSecurityType: WifiSecurityType,
  ArmStateInformation: ArmStateInformation,
  Base_CapSenseConfig: Base_CapSenseConfig,
  IPv4Configuration: IPv4Configuration,
  ActivateMapHandle: ActivateMapHandle,
  WrenchLimitation: WrenchLimitation,
  SwitchControlMapping: SwitchControlMapping,
  JointTorque: JointTorque,
  ControllerConfigurationMode: ControllerConfigurationMode,
  LedState: LedState,
  ActionHandle: ActionHandle,
  BackupEvent: BackupEvent,
  RobotEvent: RobotEvent,
  ProtectionZoneNotificationList: ProtectionZoneNotificationList,
  ProtectionZoneHandle: ProtectionZoneHandle,
  GpioEvent: GpioEvent,
  JointNavigationDirection: JointNavigationDirection,
  ProtectionZoneInformation: ProtectionZoneInformation,
  ConstrainedOrientation: ConstrainedOrientation,
  WrenchCommand: WrenchCommand,
  NetworkHandle: NetworkHandle,
  ControllerInputType: ControllerInputType,
  Admittance: Admittance,
  PreComputedJointTrajectory: PreComputedJointTrajectory,
  Twist: Twist,
  Gen3GpioPinId: Gen3GpioPinId,
  ActionNotification: ActionNotification,
  ActionType: ActionType,
  Action_action_parameters: Action_action_parameters,
  ControllerEvent: ControllerEvent,
  BridgeType: BridgeType,
  Base_ControlMode: Base_ControlMode,
  GripperMode: GripperMode,
  ChangeWrench: ChangeWrench,
  Action: Action,
  ControllerList: ControllerList,
  TrajectoryErrorReport: TrajectoryErrorReport,
  NetworkNotificationList: NetworkNotificationList,
  SequenceList: SequenceList,
  SequenceInfoNotification: SequenceInfoNotification,
  ControllerEventType: ControllerEventType,
  Base_Stop: Base_Stop,
  SequenceTaskConfiguration: SequenceTaskConfiguration,
  Base_RotationMatrix: Base_RotationMatrix,
  MapEvent_events: MapEvent_events,
  MapGroupHandle: MapGroupHandle,
  ShapeType: ShapeType,
  JointAngles: JointAngles,
  UserNotificationList: UserNotificationList,
  TwistCommand: TwistCommand,
  CartesianTrajectoryConstraint_type: CartesianTrajectoryConstraint_type,
  NavigationDirection: NavigationDirection,
  SequenceInformation: SequenceInformation,
  TransformationMatrix: TransformationMatrix,
  MapList: MapList,
  CartesianLimitation: CartesianLimitation,
  WaypointValidationReport: WaypointValidationReport,
  IKData: IKData,
  SequenceInfoNotificationList: SequenceInfoNotificationList,
  IPv4Information: IPv4Information,
  MapEvent: MapEvent,
  NetworkType: NetworkType,
  Sequence: Sequence,
  WifiConfigurationList: WifiConfigurationList,
  Faults: Faults,
  Xbox360AnalogInputIdentifier: Xbox360AnalogInputIdentifier,
  SequenceTask: SequenceTask,
  GpioConfigurationList: GpioConfigurationList,
  MappingHandle: MappingHandle,
  Snapshot: Snapshot,
  OperatingModeInformation: OperatingModeInformation,
  ControllerElementEventType: ControllerElementEventType,
  ProtectionZone: ProtectionZone,
  ActionExecutionState: ActionExecutionState,
  ConstrainedPosition: ConstrainedPosition,
  SequenceTaskHandle: SequenceTaskHandle,
  GripperCommand: GripperCommand,
  ArmStateNotification: ArmStateNotification,
  PreComputedJointTrajectoryElement: PreComputedJointTrajectoryElement,
  UserProfileList: UserProfileList,
  BridgeStatus: BridgeStatus,
  SnapshotType: SnapshotType,
  EventIdSequenceInfoNotification: EventIdSequenceInfoNotification,
  ActionEvent: ActionEvent,
  ServoingMode: ServoingMode,
  SequenceHandle: SequenceHandle,
  CartesianWaypoint: CartesianWaypoint,
  AngularWaypoint: AngularWaypoint,
  ConfigurationChangeNotificationList: ConfigurationChangeNotificationList,
  ServoingModeNotification: ServoingModeNotification,
  GripperRequest: GripperRequest,
  RequestedActionType: RequestedActionType,
  JointTorques: JointTorques,
  ServoingModeInformation: ServoingModeInformation,
  RobotEventNotification: RobotEventNotification,
  Base_SafetyIdentifier: Base_SafetyIdentifier,
  UserEvent: UserEvent,
  ControllerNotification: ControllerNotification,
  WrenchMode: WrenchMode,
  FirmwareComponentVersion: FirmwareComponentVersion,
  ConstrainedJointAngle: ConstrainedJointAngle,
  ProtectionZoneNotification: ProtectionZoneNotification,
  FullIPv4Configuration: FullIPv4Configuration,
  ServoingModeNotificationList: ServoingModeNotificationList,
  BridgeList: BridgeList,
  BridgePortConfig: BridgePortConfig,
  WifiInformation: WifiInformation,
  ConfigurationChangeNotification_configuration_change: ConfigurationChangeNotification_configuration_change,
  PasswordChange: PasswordChange,
  GpioPinConfiguration: GpioPinConfiguration,
  ControllerNotificationList: ControllerNotificationList,
  JointLimitation: JointLimitation,
  Wrench: Wrench,
  FactoryNotification: FactoryNotification,
  WristDigitalInputIdentifier: WristDigitalInputIdentifier,
  SafetyNotificationList: SafetyNotificationList,
  Waypoint_type_of_waypoint: Waypoint_type_of_waypoint,
  WifiEncryptionType: WifiEncryptionType,
  JointAngle: JointAngle,
  OperatingMode: OperatingMode,
  MapElement: MapElement,
  SequenceTasksRange: SequenceTasksRange,
  ProtectionZoneEvent: ProtectionZoneEvent,
  SequenceTasksConfiguration: SequenceTasksConfiguration,
  ControllerState: ControllerState,
  Base_Position: Base_Position,
  SystemTime: SystemTime,
  TrajectoryContinuityMode: TrajectoryContinuityMode,
  ActionList: ActionList,
  ControlModeNotificationList: ControlModeNotificationList,
  WifiConfiguration: WifiConfiguration,
  TrajectoryErrorElement: TrajectoryErrorElement,
  ControllerType: ControllerType,
  Gripper: Gripper,
  FullUserProfile: FullUserProfile,
  ConstrainedPose: ConstrainedPose,
  ChangeTwist: ChangeTwist,
  ControllerConfigurationList: ControllerConfigurationList,
  ControllerNotification_state: ControllerNotification_state,
  Map: Map,
  MapGroupList: MapGroupList,
  ControllerBehavior: ControllerBehavior,
  BridgeIdentifier: BridgeIdentifier,
  GpioCommand: GpioCommand,
  TrajectoryErrorIdentifier: TrajectoryErrorIdentifier,
  MappingInfoNotificationList: MappingInfoNotificationList,
  Ssid: Ssid,
  Waypoint: Waypoint,
  MapHandle: MapHandle,
  SignalQuality: SignalQuality,
  NetworkNotification: NetworkNotification,
  ConstrainedJointAngles: ConstrainedJointAngles,
  BridgeConfig: BridgeConfig,
  JointTrajectoryConstraintType: JointTrajectoryConstraintType,
  Base_RotationMatrixRow: Base_RotationMatrixRow,
  WifiInformationList: WifiInformationList,
  SoundType: SoundType,
  AdmittanceMode: AdmittanceMode,
  GpioPinPropertyFlags: GpioPinPropertyFlags,
  TransformationRow: TransformationRow,
  Base_CapSenseMode: Base_CapSenseMode,
  AppendActionInformation: AppendActionInformation,
  BridgeResult: BridgeResult,
  NetworkEvent: NetworkEvent,
  ControllerConfiguration: ControllerConfiguration,
  Mapping: Mapping,
  TrajectoryErrorType: TrajectoryErrorType,
  ControllerElementState: ControllerElementState,
  ZoneShape: ZoneShape,
  UserProfile: UserProfile,
  CommunicationInterfaceConfiguration: CommunicationInterfaceConfiguration,
  Base_ServiceVersion: Base_ServiceVersion,
  CartesianSpeed: CartesianSpeed,
  TrajectoryInfo: TrajectoryInfo,
  Pose: Pose,
  TwistLimitation: TwistLimitation,
  Query: Query,
  ConfigurationNotificationEvent: ConfigurationNotificationEvent,
  WaypointList: WaypointList,
  FirmwareBundleVersions: FirmwareBundleVersions,
  RobotEventNotificationList: RobotEventNotificationList,
  ControllerElementHandle_identifier: ControllerElementHandle_identifier,
  FactoryEvent: FactoryEvent,
  Timeout: Timeout,
  UserList: UserList,
  MapGroup: MapGroup,
  ConfigurationChangeNotification: ConfigurationChangeNotification,
  ControllerHandle: ControllerHandle,
  TrajectoryInfoType: TrajectoryInfoType,
  OperatingModeNotificationList: OperatingModeNotificationList,
  CartesianLimitationList: CartesianLimitationList,
  AdvancedSequenceHandle: AdvancedSequenceHandle,
  Xbox360DigitalInputIdentifier: Xbox360DigitalInputIdentifier,
  OperatingModeNotification: OperatingModeNotification,
  KinematicTrajectoryConstraints: KinematicTrajectoryConstraints,
  Delay: Delay,
  MappingList: MappingList,
  Finger: Finger,
  UserNotification: UserNotification,
  EmergencyStop: EmergencyStop,
  Base_GpioConfiguration: Base_GpioConfiguration,
  ChangeJointSpeeds: ChangeJointSpeeds,
  MappingInfoNotification: MappingInfoNotification,
  SequenceTasksPair: SequenceTasksPair,
  GpioBehavior: GpioBehavior,
  Point: Point,
  JointSpeed: JointSpeed,
  SequenceTasks: SequenceTasks,
  ActuatorInformation: ActuatorInformation,
  Base_JointSpeeds: Base_JointSpeeds,
  GpioAction: GpioAction,
  ActuatorCustomData: ActuatorCustomData,
  BaseCyclic_ServiceVersion: BaseCyclic_ServiceVersion,
  ActuatorFeedback: ActuatorFeedback,
  BaseFeedback: BaseFeedback,
  BaseCyclic_Command: BaseCyclic_Command,
  BaseCyclic_Feedback: BaseCyclic_Feedback,
  ActuatorCommand: ActuatorCommand,
  BaseCyclic_CustomData: BaseCyclic_CustomData,
  UserProfileHandle: UserProfileHandle,
  SafetyStatusValue: SafetyStatusValue,
  CountryCodeIdentifier: CountryCodeIdentifier,
  NotificationOptions: NotificationOptions,
  UARTDeviceIdentification: UARTDeviceIdentification,
  NotificationHandle: NotificationHandle,
  Connection: Connection,
  NotificationType: NotificationType,
  UARTSpeed: UARTSpeed,
  SafetyNotification: SafetyNotification,
  UARTParity: UARTParity,
  DeviceTypes: DeviceTypes,
  Permission: Permission,
  ArmState: ArmState,
  Empty: Empty,
  Unit: Unit,
  CountryCode: CountryCode,
  SafetyHandle: SafetyHandle,
  UARTWordLength: UARTWordLength,
  Timestamp: Timestamp,
  DeviceHandle: DeviceHandle,
  UARTConfiguration: UARTConfiguration,
  CartesianReferenceFrame: CartesianReferenceFrame,
  UARTStopBits: UARTStopBits,
  ControlConfig_ControlModeNotification: ControlConfig_ControlModeNotification,
  JointAccelerationSoftLimits: JointAccelerationSoftLimits,
  PayloadInformation: PayloadInformation,
  ToolConfiguration: ToolConfiguration,
  ControlConfigurationNotification: ControlConfigurationNotification,
  ControlConfig_Position: ControlConfig_Position,
  ControlConfig_ControlMode: ControlConfig_ControlMode,
  LinearTwist: LinearTwist,
  AngularTwist: AngularTwist,
  ControlConfig_ServiceVersion: ControlConfig_ServiceVersion,
  CartesianTransform: CartesianTransform,
  ControlConfig_JointSpeeds: ControlConfig_JointSpeeds,
  KinematicLimits: KinematicLimits,
  TwistLinearSoftLimit: TwistLinearSoftLimit,
  ControlConfigurationEvent: ControlConfigurationEvent,
  DesiredSpeeds: DesiredSpeeds,
  ControlConfig_ControlModeInformation: ControlConfig_ControlModeInformation,
  CartesianReferenceFrameInfo: CartesianReferenceFrameInfo,
  GravityVector: GravityVector,
  KinematicLimitsList: KinematicLimitsList,
  JointSpeedSoftLimits: JointSpeedSoftLimits,
  TwistAngularSoftLimit: TwistAngularSoftLimit,
  RebootRqst: RebootRqst,
  CalibrationItem: CalibrationItem,
  SafetyConfiguration: SafetyConfiguration,
  SerialNumber: SerialNumber,
  SafetyEnable: SafetyEnable,
  DeviceConfig_CapSenseMode: DeviceConfig_CapSenseMode,
  CalibrationElement: CalibrationElement,
  DeviceType: DeviceType,
  PowerOnSelfTestResult: PowerOnSelfTestResult,
  PartNumber: PartNumber,
  DeviceConfig_ServiceVersion: DeviceConfig_ServiceVersion,
  CalibrationParameter: CalibrationParameter,
  CalibrationParameter_value: CalibrationParameter_value,
  IPv4Settings: IPv4Settings,
  SafetyInformationList: SafetyInformationList,
  DeviceConfig_SafetyLimitType: DeviceConfig_SafetyLimitType,
  RunMode: RunMode,
  CalibrationStatus: CalibrationStatus,
  SafetyInformation: SafetyInformation,
  Calibration: Calibration,
  MACAddress: MACAddress,
  RunModes: RunModes,
  BootloaderVersion: BootloaderVersion,
  DeviceConfig_CapSenseConfig: DeviceConfig_CapSenseConfig,
  SafetyThreshold: SafetyThreshold,
  PartNumberRevision: PartNumberRevision,
  CapSenseRegister: CapSenseRegister,
  ModelNumber: ModelNumber,
  SafetyConfigurationList: SafetyConfigurationList,
  SafetyStatus: SafetyStatus,
  CalibrationResult: CalibrationResult,
  FirmwareVersion: FirmwareVersion,
  DeviceHandles: DeviceHandles,
  DeviceManager_ServiceVersion: DeviceManager_ServiceVersion,
  RobotiqGripperStatusFlags: RobotiqGripperStatusFlags,
  GripperConfig_SafetyIdentifier: GripperConfig_SafetyIdentifier,
  MotorFeedback: MotorFeedback,
  CustomDataUnit: CustomDataUnit,
  GripperCyclic_ServiceVersion: GripperCyclic_ServiceVersion,
  GripperCyclic_Command: GripperCyclic_Command,
  GripperCyclic_Feedback: GripperCyclic_Feedback,
  GripperCyclic_MessageId: GripperCyclic_MessageId,
  GripperCyclic_CustomData: GripperCyclic_CustomData,
  MotorCommand: MotorCommand,
  I2CConfiguration: I2CConfiguration,
  InterconnectConfig_ServiceVersion: InterconnectConfig_ServiceVersion,
  EthernetSpeed: EthernetSpeed,
  EthernetDevice: EthernetDevice,
  EthernetConfiguration: EthernetConfiguration,
  I2CReadRegisterParameter: I2CReadRegisterParameter,
  GPIOIdentification: GPIOIdentification,
  InterconnectConfig_GPIOConfiguration: InterconnectConfig_GPIOConfiguration,
  EthernetDuplex: EthernetDuplex,
  I2CDeviceIdentification: I2CDeviceIdentification,
  I2CReadParameter: I2CReadParameter,
  GPIOValue: GPIOValue,
  I2CRegisterAddressSize: I2CRegisterAddressSize,
  InterconnectConfig_SafetyIdentifier: InterconnectConfig_SafetyIdentifier,
  I2CDevice: I2CDevice,
  UARTPortId: UARTPortId,
  I2CMode: I2CMode,
  GPIOMode: GPIOMode,
  I2CWriteParameter: I2CWriteParameter,
  I2CData: I2CData,
  GPIOPull: GPIOPull,
  GPIOIdentifier: GPIOIdentifier,
  I2CDeviceAddressing: I2CDeviceAddressing,
  GPIOState: GPIOState,
  EthernetDeviceIdentification: EthernetDeviceIdentification,
  I2CWriteRegisterParameter: I2CWriteRegisterParameter,
  InterconnectCyclic_CustomData: InterconnectCyclic_CustomData,
  InterconnectCyclic_Command: InterconnectCyclic_Command,
  InterconnectCyclic_Feedback: InterconnectCyclic_Feedback,
  InterconnectCyclic_Feedback_tool_feedback: InterconnectCyclic_Feedback_tool_feedback,
  InterconnectCyclic_CustomData_tool_customData: InterconnectCyclic_CustomData_tool_customData,
  InterconnectCyclic_ServiceVersion: InterconnectCyclic_ServiceVersion,
  InterconnectCyclic_MessageId: InterconnectCyclic_MessageId,
  InterconnectCyclic_Command_tool_command: InterconnectCyclic_Command_tool_command,
  InterfaceModuleType: InterfaceModuleType,
  BaseType: BaseType,
  CompleteProductConfiguration: CompleteProductConfiguration,
  ProductConfigurationEndEffectorType: ProductConfigurationEndEffectorType,
  ModelId: ModelId,
  EndEffectorType: EndEffectorType,
  WristType: WristType,
  VisionModuleType: VisionModuleType,
  ArmLaterality: ArmLaterality,
  Option: Option,
  OptionInformation: OptionInformation,
  OptionValue: OptionValue,
  VisionEvent: VisionEvent,
  FocusPoint: FocusPoint,
  VisionConfig_ServiceVersion: VisionConfig_ServiceVersion,
  FocusAction: FocusAction,
  ManualFocus: ManualFocus,
  VisionConfig_RotationMatrix: VisionConfig_RotationMatrix,
  VisionNotification: VisionNotification,
  Sensor: Sensor,
  IntrinsicProfileIdentifier: IntrinsicProfileIdentifier,
  DistortionCoefficients: DistortionCoefficients,
  Resolution: Resolution,
  OptionIdentifier: OptionIdentifier,
  SensorFocusAction: SensorFocusAction,
  BitRate: BitRate,
  TranslationVector: TranslationVector,
  SensorFocusAction_action_parameters: SensorFocusAction_action_parameters,
  FrameRate: FrameRate,
  SensorIdentifier: SensorIdentifier,
  SensorSettings: SensorSettings,
  VisionConfig_RotationMatrixRow: VisionConfig_RotationMatrixRow,
  IntrinsicParameters: IntrinsicParameters,
  ExtrinsicParameters: ExtrinsicParameters,
  FollowCartesianTrajectoryActionFeedback: FollowCartesianTrajectoryActionFeedback,
  FollowCartesianTrajectoryAction: FollowCartesianTrajectoryAction,
  FollowCartesianTrajectoryActionResult: FollowCartesianTrajectoryActionResult,
  FollowCartesianTrajectoryGoal: FollowCartesianTrajectoryGoal,
  FollowCartesianTrajectoryResult: FollowCartesianTrajectoryResult,
  FollowCartesianTrajectoryActionGoal: FollowCartesianTrajectoryActionGoal,
  FollowCartesianTrajectoryFeedback: FollowCartesianTrajectoryFeedback,
};
