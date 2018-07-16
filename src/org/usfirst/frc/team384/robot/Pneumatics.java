package org.usfirst.frc.team384.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;

public class Pneumatics {

	// Compressor
	private Compressor compressor = new Compressor(0);
	
	
	public boolean isPressurized() {
	    if (compressor.enabled()) {
	      return false;
	    } else {
	      return true;
	    }
	  }

	  public void setSol(Solenoid sol, boolean state) {
	    sol.set(state);
	  }

	  public void disableCompressor() {
	    compressor.stop();
	  }

	  public void enableCompressor() {
	    compressor.start();
	  }

	  public double getCompressorCurrent() {
	    return compressor.getCompressorCurrent();
	  }

	  public void compressorSetClosedLoop(boolean state)	{
		  compressor.setClosedLoopControl(state);
	  }
	  
}
