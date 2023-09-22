/*
 ____  _____ _        _    
| __ )| ____| |      / \   
|  _ \|  _| | |     / _ \  
| |_) | |___| |___ / ___ \ 
|____/|_____|_____/_/   \_\

http://bela.io

C++ Real-Time Audio Programming with Bela - Lecture 20: Phase vocoder, part 3
fft-robotisation: zero the phase for each FFT bin, producing a robot voice effect
*/

#include <Bela.h>
#include <libraries/Fft/Fft.h>
#include <libraries/Scope/Scope.h>
#include <libraries/Gui/Gui.h>
#include <libraries/GuiController/GuiController.h>
#include <libraries/Trill/Trill.h>
#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>
#include "MonoFilePlayer.h"


// FFT-related variables
Fft gFftMag;
Fft gFftPhi;
Fft gFftRect;// FFT processing object
const int gFftSize = 2048;	// FFT window size in samples
int gHopSize = 256;	// How often we calculate a window
float gHopSizeRange[2] = {50.0, 200.0}; 

// Circular buffer and pointer for assembling a window of samples
const int gBufferSize = 16384;
std::vector<float> gInputBufferMag;
std::vector<float> gInputBufferPhi;
int gInputBufferPointer = 0;
int gHopCounter = 0;

// Circular buffer for collecting the output of the overlap-add process
std::vector<float> gOutputBuffer;
int gOutputBufferWritePointer = gFftSize + gHopSize;
int gOutputBufferReadPointer = 0;

// Buffer to hold the windows for FFT analysis and synthesis
std::vector<float> gAnalysisWindowBuffer;
std::vector<float> gSynthesisWindowBuffer;

// Name of the sound file (in project folder)
std::string gFilenameMag = "theme.wav"; 
std::string gFilenamePhi = "voice.wav"; 

// Object that handles playing sound from a buffer
MonoFilePlayer gPlayerMag;
MonoFilePlayer gPlayerPhi;

// Thread for FFT processing
AuxiliaryTask gFftTask;
int gCachedInputBufferPointer = 0;

void process_fft_background(void *);

// Bela oscilloscope
Scope gScope;

// Browser-based GUI to adjust parameters
Gui gGui;
GuiController gGuiController;

// Create Trill Square Object
Trill touchSensor;
float gTouchPosition[2] = {0.0, 0.0}; // Horizontal and Vertical position for Trill
float gTouchSize = 0.0; // Touch size

//Sample amplitude 
float gAmp = 0.0; 

//
std::vector<float> magInput(gFftSize);
std::vector<float> phasInput(gFftSize);
std::vector<float> outPutMag(gFftSize);
std::vector<float> outPutPhi(gFftSize);


void loop(void*) {
	// This is the auxilary task function that will read the Trill sensor's loop
	while(!Bela_stopRequested())
	{
		//Read locations from Trill sensor 
		touchSensor.readI2C();
		gTouchSize = touchSensor.compoundTouchSize();
		gTouchPosition[0] = touchSensor.compoundTouchHorizontalLocation();
		// gTouchPosition[1] = touchSensor.compoundTouchLocation();
		//Put the process to sleep when finished 
		usleep(12000);
	}
}


bool setup(BelaContext *context, void *userData)
{
	//Set up Trill Square
	//Setup as (bus number, Trill type, default address), if success, will return 0. 
	if(touchSensor.setup(1, Trill::SQUARE) != 0){
		fprintf(stderr, "Unable to initialise touch sensor\n");
		return false;
	}
	Bela_runAuxiliaryTask(loop);// create and start the aux task 
	
	// Load the audio file
	if(!gPlayerMag.setup(gFilenameMag)) {
    	rt_printf("Error loading audio file '%s'\n", gFilenameMag.c_str());
    	return false;
	}
	if(!gPlayerPhi.setup(gFilenamePhi)) {
    	rt_printf("Error loading audio file '%s'\n", gFilenamePhi.c_str());
    	return false;
	}

	// Print some useful info
	rt_printf("Loaded the audio file '%s' with %d frames (%.1f seconds)\n", 
    			gFilenameMag.c_str(), gPlayerMag.size(),
    			gPlayerMag.size() / context->audioSampleRate);
    rt_printf("Loaded the audio file '%s' with %d frames (%.1f seconds)\n", 
    			gFilenamePhi.c_str(), gPlayerPhi.size(),
    			gPlayerPhi.size() / context->audioSampleRate);
	
	// Set up the FFT and its buffers
	gFftMag.setup(gFftSize);
	gFftPhi.setup(gFftSize);
	gFftRect.setup(gFftSize);
	gInputBufferMag.resize(gBufferSize);
	gInputBufferPhi.resize(gBufferSize);
	gOutputBuffer.resize(gBufferSize);
	
	//pre-compute Hann window for Analysis 
	gAnalysisWindowBuffer.resize(gFftSize);
	for(int n = 0; n < gFftSize; n++) {
		// Hann window
		gAnalysisWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gFftSize - 1)));
	}
	//pre-compute Hann window for synthesis 
	gSynthesisWindowBuffer.resize(gFftSize);
	for(int n = 0; n < gFftSize; n++) {
		// Hann window
		gSynthesisWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(gFftSize - 1)));
	}
	
	// Initialise the scope
	//gScope.setup(2, context->audioSampleRate);
	
	//Initialise GUi 
	// gGui.setup(context->projectName);
	// gGuiController.setup(&gGui, "Robotisation Controller");
	//Add Gui Controller 
	//gGuiController.addSlider("Hop Size", 256, 64, 1024, 1); 
	
	// Set up the thread for the FFT
	gFftTask = Bela_createAuxiliaryTask(process_fft_background, 50, "bela-process-fft");

	return true;
}

// This function handles the FFT processing in this example once the buffer has
// been assembled.

//Cross Synthesis 
void crosspec(std::vector<float>& magInput, std::vector<float>& phasInput, std::vector<float>& outPutMag,std::vector<float>& outPutPhi, int fftSize){
	
	int i;
	float mag, phi;
	magInput.resize(fftSize);
	phasInput.resize(fftSize);
	outPutMag.resize(fftSize);
	outPutPhi.resize(fftSize);
	//take care of real-valued points at 0hz and Nyquist
	outPutMag[0] = magInput[0];
	outPutMag[1] = magInput[1];
	
	for(i = 2; i < fftSize; i++){
		//get the magnitudes if one input
		mag = (float) sqrt(magInput[i] * magInput[i] + magInput[i+1]* magInput[i+1]);
		//get the phase of the other
		phi = (float) atan2(phasInput[i+1], phasInput[i]);
		//combine them and convert to rectangular form
		outPutMag[i] = (float)(mag*cos(phi));
		outPutPhi[i] = (float)(mag*sin(phi));
	}
}

void process_fft(std::vector<float> const& inBufferMag,std::vector<float> const& inBufferPhi, unsigned int inPointer, std::vector<float>& outBuffer, unsigned int outPointer)
{
	static std::vector<float> unwrappedBufferMag(gFftSize);
	static std::vector<float> unwrappedBufferPhi(gFftSize);// Container to hold the unwrapped values
	
	// Copy buffer into FFT input 
	for(int n = 0; n < gFftSize; n++) {
		// Use modulo arithmetic to calculate the circular buffer index
		int circularBufferIndex = (inPointer + n - gFftSize + gBufferSize) % gBufferSize;
		unwrappedBufferMag[n] = inBufferMag[circularBufferIndex] * gAnalysisWindowBuffer[n];
		unwrappedBufferPhi[n] = inBufferPhi[circularBufferIndex] * gAnalysisWindowBuffer[n];
		
	}
	
	// Process the FFT based on the time domain input
	gFftMag.fft(unwrappedBufferMag);
	gFftPhi.fft(unwrappedBufferPhi);
	
	for(int i = 0; i< gFftSize; i++){
		magInput[i] = gFftMag.fdr(i);
		phasInput[i] = gFftPhi.fdi(i);
	}
	
	crosspec(magInput, phasInput, outPutMag,outPutPhi, gFftSize);
	gFftRect.ifft(outPutMag, outPutPhi);
	// Robotise the output
	// for(int n = 0; n < gFftSize; n++) {
		
	// 	// float amplitude = gFft.fda(n);
	// 	// gFft.fdr(n) = amplitude;
	// 	// gFft.fdi(n) = 0;
	// }
		
	// Run the inverse FFT
	//gFftRect.ifft();

	
	// Add timeDomainOut into the output buffer
	for(int n = 0; n < gFftSize; n++) {
		int circularBufferIndex = (outPointer + n - gFftSize + gBufferSize) % gBufferSize;
		outBuffer[circularBufferIndex] += gFftRect.td(n) * gSynthesisWindowBuffer[n];
	}
}

// This function runs in an auxiliary task on Bela, calling process_fft
void process_fft_background(void *)
{
	process_fft(gInputBufferMag,gInputBufferPhi,  gCachedInputBufferPointer, gOutputBuffer, gOutputBufferWritePointer);

	// Update the output buffer write pointer to start at the next hop
	gOutputBufferWritePointer = (gOutputBufferWritePointer + gHopSize) % gBufferSize;
}

void recalculate_window(unsigned int length){
	if(length > gAnalysisWindowBuffer.size()){
		length = gAnalysisWindowBuffer.size();
	}
	if(length > gSynthesisWindowBuffer.size()){
		length = gSynthesisWindowBuffer.size();
	}
	for(int n =0; n < length; n++){
		gAnalysisWindowBuffer[n] = 0.5f * (1.0f - cosf(2.0 * M_PI * n / (float)(length - 1)));
		gSynthesisWindowBuffer[n] = gAnalysisWindowBuffer[n]; 
	}
}

void render(BelaContext *context, void *userData)
{	
	// if hope size changes, recalculate the window to an overlap factor of 4 
	int hopSize = map(gTouchPosition[0], 0.0, 1.0, int(gHopSizeRange[0]), int(gHopSizeRange[1]));
	if(hopSize != gHopSize){
		int newLength = hopSize*4;
		if(newLength > gFftSize){
			newLength = gFftSize;
		} 
		recalculate_window(newLength);
		
		gHopSize = hopSize;
	}
	
	for(unsigned int n = 0; n < context->audioFrames; n++) {
        // Read the next sample from the buffer
        float gAmp = gTouchSize * 20.0;
        float inMag = gPlayerMag.process() * gAmp;
        float inPhi = gPlayerPhi.process() * gAmp;
        

		// Store the sample ("in") in a buffer for the FFT
		// Increment the pointer and when full window has been 
		// assembled, call process_fft()
		gInputBufferMag[gInputBufferPointer] = inMag;
		gInputBufferPhi[gInputBufferPointer] = inPhi;
		if(++gInputBufferPointer >= gBufferSize) {
			// Wrap the circular buffer
			// Notice: this is not the condition for starting a new FFT
			gInputBufferPointer = 0;
		}
	
		
		// Get the output sample from the output buffer
		float out = gOutputBuffer[gOutputBufferReadPointer];
		
		// Then clear the output sample in the buffer so it is ready for the next overlap-add
		gOutputBuffer[gOutputBufferReadPointer] = 0;
		
		// Scale the output down by the overlap factor (e.g. how many windows overlap per sample?)
		out *= (float)gHopSize / (float)gFftSize;
		
		// Increment the read pointer in the output cicular buffer
		gOutputBufferReadPointer++;
		if(gOutputBufferReadPointer >= gBufferSize)
			gOutputBufferReadPointer = 0;
		
		// Increment the hop counter and start a new FFT if we've reached the hop size
		if(++gHopCounter >= gHopSize) {
			gHopCounter = 0;
			
			gCachedInputBufferPointer = gInputBufferPointer;
			Bela_scheduleAuxiliaryTask(gFftTask);
		}

		// Write the audio to the output
		for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
			audioWrite(context, n, channel, out);
		}
		
		// Log to the scope
		//gScope.log(in, out);
	}
}

void cleanup(BelaContext *context, void *userData)
{

}
