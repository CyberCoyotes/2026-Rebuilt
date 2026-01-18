package frc.robot.subsystems.vision;


    public interface VisionIO {
    void updateInputs(VisionIOInputs inputs);
    void setPipeline(int pipelineIndex);
    void setLEDMode(LEDMode mode);
    
    enum LEDMode {
        PIPELINE_DEFAULT(0),
        FORCE_OFF(1),
        FORCE_BLINK(2),
        FORCE_ON(3);
        
        public final int value;
        LEDMode(int value) { this.value = value; }
    }
    
    class VisionIOInputs {
        // ... your existing fields
    }
}
