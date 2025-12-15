package frc.robot.subsystems;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
// The ElasticSubsystem periodically publishes custom data to our NetworkTable for Elastic

public class ElasticSubsystem extends SubsystemBase{
    private final NetworkTable elasticTable;
    private final DoublePublisher m_taPub;
    private final DoublePublisher m_txPub;
    private final DoublePublisher m_tyPub;


    public ElasticSubsystem() {
        elasticTable = NetworkTableInstance.getDefault().getTable("elastic_datatable");
        m_taPub = elasticTable.getDoubleTopic("ta").publish();
        m_txPub = elasticTable.getDoubleTopic("tx").publish();
        m_tyPub = elasticTable.getDoubleTopic("ty").publish();
    }

    @Override
    public void periodic() {
        m_taPub.set(LimelightHelpers.getTA("limelight"));
        m_txPub.set(LimelightHelpers.getTX("limelight"));
        m_tyPub.set(LimelightHelpers.getTY("limelight"));
    }
}
