package frc.robot.Subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

public class Indexer{
    private final IndexerIO indexerIO;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private IndexerStates indexerState = IndexerStates.IDLE;
    private double indexerVoltage = 0;

    public enum IndexerStates{
        IDLE,
        HANDOFF
    }
    
    public Indexer(IndexerIO indexerIO) {
        this.indexerIO = indexerIO;
    }

  
    public void Loop(){
        indexerIO.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput("IndexerState", this.indexerState);
        switch(indexerState){
            case IDLE:
                indexerIO.setIndexerVoltage(0);
                break;
            case HANDOFF:
                indexerIO.setIndexerVoltage(indexerVoltage);
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(IndexerStates.IDLE);
    }

    public void requestHandoff(double indexerVoltage){
        this.indexerVoltage = indexerVoltage;
        setState(IndexerStates.HANDOFF);
    }

    public void setState(IndexerStates nextState){
        this.indexerState = nextState;
    }

    public double getIndexerCurrent(){
        return inputs.indexerCurrent;
    }

    public IndexerStates getIndexerState(){
        return this.indexerState;
    }
}