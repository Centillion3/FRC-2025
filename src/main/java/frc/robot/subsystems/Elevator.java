package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.ElevatorState;

public class Elevator extends SubsystemBase{
    private SparkFlex motor;
    private boolean zeroMode;

    private ElevatorState currentState;

    public enum ElevatorEvent {
        GO_INTAKE,
        GO_L1,
        GO_L2,
        GO_L3,
        GO_L4
    }

    private Map<ElevatorState, Map<ElevatorEvent, ElevatorState>> transitions;

    public Elevator(){
        motor = new SparkFlex(Constants.Elevator.elevatorMotorID, MotorType.kBrushless);
        currentState = ElevatorState.INTAKE;
        transitions = new EnumMap<>(ElevatorState.class);
        initializeTransitions();
        setGoalState(ElevatorState.INTAKE);
    }

    private void initializeTransitions() {
        for (ElevatorState state : ElevatorState.values()) {
            transitions.put(state, new EnumMap<>(ElevatorEvent.class));
            transitions.get(state).put(ElevatorEvent.GO_INTAKE, ElevatorState.INTAKE);
            transitions.get(state).put(ElevatorEvent.GO_L1, ElevatorState.L1);
            transitions.get(state).put(ElevatorEvent.GO_L2, ElevatorState.L2);
            transitions.get(state).put(ElevatorEvent.GO_L3, ElevatorState.L3);
            transitions.get(state).put(ElevatorEvent.GO_L4, ElevatorState.L4);
        }
    }

    public void handleEvent(ElevatorEvent event){
        Map<ElevatorEvent, ElevatorState> stateTransitions = transitions.get(currentState);
        if(stateTransitions != null && stateTransitions.containsKey(event)){
            currentState = stateTransitions.get(event);
            setGoalState(currentState);
        }
    }

    public void setGoalState(ElevatorState desiredState){
        Constants.Elevator.elevatorPID.reset(getHeight());
        Constants.Elevator.elevatorPID.setGoal(desiredState.height);
        SmartDashboard.putNumber("desiredPosition", desiredState.height);
    }

    @Override
    public void periodic(){
        if(!zeroMode && DriverStation.isEnabled()){
            double PIDOutput = Constants.Elevator.elevatorPID.calculate(getHeight());
            double FFOutput = Constants.Elevator.elevatorFF.calculate(Constants.Elevator.elevatorPID.getSetpoint().velocity);
            motor.setVoltage((PIDOutput + FFOutput));
            if(motor.getReverseLimitSwitch().isPressed()){
                motor.getEncoder().setPosition(0);
            }
        }
        else{
            motor.set(-0.2);
            if(motor.getReverseLimitSwitch().isPressed()){
                zeroMode = false;
                motor.getEncoder().setPosition(0);
                setGoalState(ElevatorState.INTAKE);
                currentState = ElevatorState.INTAKE;
            }
        }
        SmartDashboard.putNumber("encoderPosition", motor.getEncoder().getPosition());
        SmartDashboard.putNumber("height", getHeight());
        SmartDashboard.putNumber("percent", motor.getAppliedOutput());
        SmartDashboard.putBoolean("limit switch", motor.getReverseLimitSwitch().isPressed());
    }

    public void toggleZeroElevator(){
        zeroMode = !zeroMode;
    }

    public double getHeight(){
        return motor.getEncoder().getPosition() / Constants.Elevator.rotationsPerMeter;
    }

    public class ChangeState extends Command{
        private ElevatorEvent event;
        private boolean continuous;

        public ChangeState(ElevatorEvent event){
            this.event = event;
            addRequirements(Elevator.this);
        }

        @Override
        public void initialize(){
            Elevator.this.handleEvent(event);
        }

        @Override
        public boolean isFinished(){
            return !continuous && MathUtil.isNear(getHeight(), Constants.Elevator.elevatorPID.getGoal().position, Constants.Elevator.elevatorTolerance);
        }
    }
}