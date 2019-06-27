
#include "Edge.hpp"

#include "Edge_Component.hpp"
#include <iostream>
#include <fstream>
#include <string>

#define DR_WAV_IMPLEMENTATION
#include "dep/dr_wav.h"


using namespace std;


template <int OVERSAMPLE, int QUALITY>
struct VoltageControlledOscillator {

	float lastSyncValue = 0.0f;
	float phase = 0.0f;
	float freq;
	float pw = 0.6f;
	float pitch;
	//
	float width;
	float widthCv;
	float _dual;
	float al_window = 0.5f;
	float ar_window = 0.5f;
	float bl_window = 0.5f;
	float br_window = 0.5f;
	float faded_wavfr,faded_wavre = 0.0f;
	//
	bool syncEnabled = false;
	bool syncDirection = false;
	bool invert = false;
    bool tab_loaded = false;
	//FILE * temp_file_out = NULL;
    //FILE * wave_f = NULL;
	short temp_buf[256]={0};
    float mid_phase = 0.0f;
	float buf_wavefront[256]={0};
	float buf_waverear[256]={0};
	float buf_final[257]={0};


    std::string plug_directory = asset::plugin(pluginInstance, "res/waves/");
	float wave[64][256]={{0}};
    const std::string wavefiles[64]={"00.wav","01.wav","02.wav","03.wav","04.wav","05.wav","06.wav","07.wav","08.wav","09.wav","10.wav","11.wav","12.wav","13.wav","14.wav","15.wav","16.wav","17.wav","18.wav","19.wav","20.wav","21.wav","22.wav","23.wav","24.wav","25.wav","26.wav","27.wav","28.wav","29.wav","30.wav","31.wav","32.wav","33.wav","34.wav","35.wav","36.wav","37.wav","38.wav","39.wav","40.wav","41.wav","42.wav","43.wav","44.wav","45.wav","46.wav","47.wav","48.wav","49.wav","50.wav","51.wav","52.wav","53.wav","54.wav","55.wav","56.wav","57.wav","58.wav","59.wav","60.wav","61.wav","62.wav","63.wav"};
	dsp::Decimator<OVERSAMPLE, QUALITY> sinDecimator;

	dsp::RCFilter sqrFilter;

	// For analog detuning effect
	float pitchSlew = 0.0f;
	int pitchSlewIndex = 0;

	float sinBuffer[OVERSAMPLE] = {};



    void LoadWaves(){

        for(int j=0; j<64; j++){
            std::string file_name = plug_directory+wavefiles[j];
            const char *chemin = file_name.c_str();

            unsigned int channels;
            unsigned int sampleRate;
            drwav_uint64 totalPCMFrameCount;
            float* pSampleData = drwav_open_file_and_read_pcm_frames_f32(chemin, &channels, &sampleRate, &totalPCMFrameCount);

            //Normalisation

            double max_value = 0.0f;
            for(int i = 0; i<256 ; i++){
                    max_value = std::max(max_value,abs(pSampleData[i]/2.0));
            }

            for(int i = 0; i<256 ; i++){
                wave[j][i] = pSampleData[i]/2.0f;
                wave[j][i] = wave[j][i]*(1/max_value);
                //wave[j][i] = max_value/5;
            }

            drwav_free(pSampleData);;
        }
        tab_loaded = true;
    }



// Il va falloir clamp PITCH !

	void setPitch(float pitchKnob, float pitchCv, float _lfo_param) {
		// Compute frequency
		pitch = pitchKnob;
		/*if (analog) {
			// Apply pitch slew
			const float pitchSlewAmount = 3.0f;
			pitch += pitchSlew * pitchSlewAmount;
		}
		else {
        */
			// Quantize coarse knob if digital mode
			pitch = roundf(pitch);
		//}
		pitch += pitchCv;
		// Note C4
		freq = 261.626f * powf(2.0f, pitch / 12.0f);

        freq = clamp(freq,1.0f,10000.0f);
        if(_lfo_param == 0){
          freq = freq/100;
		}


	}

	void setInvert(float _invert){
        if(_invert<0.5f){
            invert = false;
        }
        else{
            invert = true;
        }

	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	void setWaves(float _wavefront,float _waverear,int auto_scale){

	    _wavefront*=63;
	    _waverear*=63;

	    float dif_fr = _wavefront - faded_wavfr;
	    if(dif_fr>0){
            _wavefront += std::min(dif_fr,0.01f);
	    }
        if(dif_fr<0){
            _wavefront += std::max(dif_fr,-0.01f);
        }
	    float dif_re = _waverear - faded_wavre;
	    if(dif_re>0){
            _waverear += std::min(dif_re,0.01f);
	    }
        if(dif_fr<0){
            _waverear += std::max(dif_re,-0.01f);
        }


	    if(_wavefront>63.0f){_wavefront=63.0f;}
	    if(_waverear>63.0f){_waverear=63.0f;}
	    if(_wavefront<=0.0f){_wavefront=0.00f;}
	    if(_waverear<=0.0f){_waverear=0.00f;}
	    float coef1f = _wavefront-int(_wavefront);
	    float coef2f = 1 - coef1f;
        float coef1r = _waverear-int(_waverear);
	    float coef2r = 1 - coef1r;
	    float max_wav_f, max_wav_r = 0.0f;
        for(int i = 0; i<=255; i++){

            if(_wavefront<63){
                buf_wavefront[i] = coef2f*wave[int(_wavefront)][i]+coef1f*wave[int(_wavefront)+1][i];
            }
            else{
                buf_wavefront[i] = wave[int(_wavefront)][i];
            }
            if(_waverear<63){
                buf_waverear[i] = coef2r*wave[int(_waverear)][i]+coef1r*wave[int(_waverear)+1][i];
            }
            else{
                buf_waverear[i] = wave[int(_waverear)][i];
            }
            max_wav_f = std::max(max_wav_f,abs(buf_wavefront[i]));
            max_wav_r = std::max(max_wav_r,abs(buf_waverear[i]));
        }

        //RESCALING ->
        if(auto_scale==1){
            for(int i = 0; i<=255; i++){
                buf_wavefront[i] = buf_wavefront[i]*(1/max_wav_f);
                buf_waverear[i] = buf_waverear[i]*(1/max_wav_r);
            }
        }


        faded_wavfr = _wavefront;
        faded_wavre = _waverear;
	}


	void setPulseWidth(float pulseWidth) {
		const float pwMin = 0.01f;
		pw = clamp(pulseWidth, pwMin, 1.0f - pwMin);
	}

	//
	void setWidth(float widthKnob,float widthCv, float dual) {
        width = widthKnob;
        width += (widthCv/10);
        if(width>1){width=1.0f;}
        if(width<0){width=0;}
        _dual = 1-dual;

        if(_dual < 0.5f){
            al_window = 0.5f - (width/2);
            ar_window = 0.5f + (width/2);
        }
        else{
            al_window = 0.25f - (width/4);
            ar_window = 0.25f + (width/4);
            bl_window = 0.75f - (width/4);
            br_window = 0.75f + (width/4);
        }
  	}


	void process(float deltaTime, float syncValue) {

		float deltaPhase = clamp(freq * deltaTime, 1e-6, 0.5f);

		// Detect sync
		int syncIndex = -1; // Index in the oversample loop where sync occurs [0, OVERSAMPLE)
		float syncCrossing = 0.0f; // Offset that sync occurs [0.0f, 1.0f)
		if (syncEnabled) {
			syncValue -= 0.01f;
			if (syncValue > 0.0f && lastSyncValue <= 0.0f) {
				float deltaSync = syncValue - lastSyncValue;
				syncCrossing = 1.0f - syncValue / deltaSync;
				syncCrossing *= OVERSAMPLE;
				syncIndex = (int)syncCrossing;
				syncCrossing -= syncIndex;
			}
			lastSyncValue = syncValue;
		}

		if (syncDirection)
			deltaPhase *= -1.0f;




        int i = int(phase*255)+3;
        if(i>=255.0f){
            i -= 255.0f;
        }

            if(invert){
               if(_dual < 0.5f){
                    if( (phase > al_window and phase < ar_window) or (al_window == 0.0f) ) {
                        if(i==255){
                            buf_final[i] = (buf_waverear[0]+buf_waverear[255])/2;
                        }
                        else{
                            if(phase==al_window or  phase==ar_window){
                                buf_final[i] = (buf_waverear[255-i]+buf_wavefront[i])/2;
                            }
                            else{
                                buf_final[i] = buf_waverear[255-i];
                            }
                        }
                    }
                    else{
                        if(i==255){
                            buf_final[i] = (buf_wavefront[0]+buf_wavefront[255])/2;
                        }
                        else{
                            buf_final[i] = buf_wavefront[i];
                        }
                    }
               }
               else{
                    if( (phase > al_window and phase < ar_window) or (phase > bl_window and phase < br_window) or ( al_window == 0.0f and ar_window == 0.5f)) {
                        if(i==255){
                            buf_final[i] = (buf_waverear[0] + buf_waverear[255])/2;
                        }
                        else{
                            if(phase==al_window or  phase==ar_window){
                                buf_final[i] = (buf_waverear[255-i]+buf_wavefront[i])/2;
                            }
                            else{
                                buf_final[i] = buf_waverear[255-i];
                            }
                        }
                    }
                   else{
                        if(i==255){
                            buf_final[i] = (buf_wavefront[0]+buf_wavefront[255])/2;
                        }
                        else{
                            buf_final[i] = buf_wavefront[i];
                        }
                   }
               }
            }
            else{
                if(_dual < 0.5f){
                    if((phase > al_window and phase < ar_window) or (al_window == 0.0f and ar_window == 1.0f)) {
                        if(i==255){
                            buf_final[i] = (buf_waverear[0]+buf_waverear[255])/2;
                        }
                        else{
                            if(phase==al_window or  phase==ar_window){
                                buf_final[i] = (buf_waverear[i]+buf_wavefront[i])/2;
                            }
                            else{
                                buf_final[i] = buf_waverear[i];
                            }
                        }
                    }
                    else{
                        if(i==255){
                            buf_final[i] = (buf_wavefront[0]+buf_wavefront[255])/2;
                        }
                        else{
                            buf_final[i] = buf_wavefront[i];
                        }
                    }
                }
                else{
                    if( (phase > al_window and phase < ar_window) or (phase > bl_window and phase < br_window) or ( al_window == 0.0f and ar_window == 1.0f)) {
                        if(i==255){
                            buf_final[i] = (buf_waverear[0]+buf_waverear[255])/2;
                        }
                        else{
                            if(phase==al_window or  phase==ar_window){
                                buf_final[i] = (buf_waverear[i]+buf_wavefront[i])/2;
                            }
                            else{
                                buf_final[i] = buf_waverear[i];
                            }
                        }
                    }
                   else{
                        if(i==255){
                            buf_final[i] = (buf_wavefront[0]+buf_wavefront[255])/2;
                        }
                        else{
                            buf_final[i] = buf_wavefront[i];
                        }
                   }
                }
            }
        //}

        sqrFilter.setCutoff(44100.0f * deltaTime);




        for (int i = 0; i < OVERSAMPLE; i++) {
            if (syncIndex == i) {
					phase = 0.0f;
			}

			// Advance phase
            sinBuffer[i]= interpolateLinear(buf_final, phase*255.0f) ;
            sqrFilter.process(sinBuffer[i]);
            sinBuffer[i]=sqrFilter.lowpass();
            phase += deltaPhase / OVERSAMPLE;
			while (phase > 1.0f) {
                phase -= 1.0f;
            }
            while (phase < 0) {
                phase += 1.0f;
            }



		}

	}

	float sin() {
		return sinDecimator.process(sinBuffer);
	}

	float light() {
		return sinf(2*M_PI * phase);
	}
};


struct WCO_Osc : Module {
	enum ParamIds {
		MODE_PARAM,
		INVERT_PARAM,
		LFO_NOISE_PARAM,
		FRONT_PARAM,
        WIDTH_PARAM,
		REAR_PARAM,
		CV_FRONT_PARAM,
        CV_WIDTH_PARAM,
		CV_REAR_PARAM,
		FREQ_PARAM,
		FINE_PARAM,
		FM_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
	    FM_INPUT,
	    SYNK_INPUT,
		PITCH_INPUT,
		FRONT_INPUT,
		WIDTH_INPUT,
		REAR_INPUT,
		NUM_INPUTS
	};
	enum OutputIds {
		OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds {
		//PHASE_POS_LIGHT,
	//	PHASE_NEG_LIGHT,
		NUM_LIGHTS
	};

	VoltageControlledOscillator<16, 16> oscillator;
	FILE * wave_f = NULL;
	float  BUFFER[256]={0};
	float l_FRONT_PARAM=1.0f;
	float l_WIDTH_PARAM;
	float l_REAR_PARAM;
	float l_CV_FRONT_PARAM;
	float l_CV_REAR_PARAM;
	float l_CV_WIDTH_PARAM;
	float l_INVERT_PARAM;
	float l_FRONT_INPUT;
	float l_REAR_INPUT;
	float l_WIDTH_INPUT;
	float l_MODE_PARAM;
	float l_FM_INPUT;

	int lfo_range = 0;
	int autoscale = 0;

	WCO_Osc() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
		configParam(MODE_PARAM, 0.0f, 1.0f, 1.0f);
        configParam(INVERT_PARAM, 0.0f, 1.0f, 1.0f);
        configParam(LFO_NOISE_PARAM, 0.0f, 1.0f, 1.0f);
        configParam(FRONT_PARAM, 0.0f, 1.0f, 0.0f);
        configParam(WIDTH_PARAM, 0.0f, 1.0f, 0.0f);
        configParam(REAR_PARAM, 0.0f, 1.0f, 0.0f);
        configParam(CV_FRONT_PARAM, 0.0f, 1.0f, 0.0f);
        configParam(CV_WIDTH_PARAM, 0.0f, 1.0f, 0.0f);
        configParam(CV_REAR_PARAM, 0.0f, 1.0f, 0.0f);
        configParam(FREQ_PARAM, -54.0f, 54.0f, 0.0f);
        configParam(FINE_PARAM, -1.0f, 1.0f, 0.0f);
        configParam(FM_PARAM, 0.0f, 1.0f, 0.0f);



	}
	void process(const ProcessArgs &args) override;
};


void WCO_Osc::process(const ProcessArgs &args)  {





    if(oscillator.tab_loaded == false){
        oscillator.LoadWaves();
    }
    float pitchFine = 3.0f * dsp::quadraticBipolar(params[FINE_PARAM].getValue());



    float pitchCv = 12.0f * inputs[PITCH_INPUT].getVoltage();


    if (inputs[FM_INPUT].active || inputs[FM_INPUT].getVoltage() != l_FM_INPUT ) {
        pitchCv += dsp::quadraticBipolar(params[FM_PARAM].getValue()) * 12.0f * inputs[FM_INPUT].getVoltage();
    }
    oscillator.setPitch(params[FREQ_PARAM].getValue(), pitchFine + pitchCv,params[LFO_NOISE_PARAM].getValue());
    oscillator.setPulseWidth(0.5f);//oscillator.setPulseWidth(params[PW_PARAM].value + params[WIDTH_PARAM].value * inputs[PW_INPUT].value / 10.0f);
    if(inputs[SYNK_INPUT].active)
        oscillator.syncEnabled = 1;
    else
        oscillator.syncEnabled = 0;


    oscillator.setInvert(params[INVERT_PARAM].getValue());


    if( (params[WIDTH_PARAM].getValue() != l_WIDTH_PARAM) || (inputs[WIDTH_INPUT].getVoltage() != l_WIDTH_INPUT) || (params[CV_WIDTH_PARAM].getValue() != l_WIDTH_PARAM) || (params[MODE_PARAM].getValue() != l_MODE_PARAM) ){
        oscillator.setWidth( params[WIDTH_PARAM].getValue(),(inputs[WIDTH_INPUT].getVoltage()*params[CV_WIDTH_PARAM].getValue()),params[MODE_PARAM].getValue()) ;
    }

    if( ( params[FRONT_PARAM].getValue() != l_FRONT_PARAM ) || (params[CV_FRONT_PARAM].getValue() != l_CV_FRONT_PARAM ) || (inputs[FRONT_INPUT].getVoltage() != l_FRONT_INPUT) || ( params[REAR_PARAM].getValue() != l_REAR_PARAM ) || (params[CV_REAR_PARAM].getValue() != l_CV_REAR_PARAM ) || (inputs[REAR_INPUT].getVoltage() != l_REAR_INPUT) || oscillator.tab_loaded == false ){
        oscillator.setWaves( params[FRONT_PARAM].getValue()+(params[CV_FRONT_PARAM].getValue()*(inputs[FRONT_INPUT].getVoltage()/5)),params[REAR_PARAM].getValue()+(params[CV_REAR_PARAM].getValue()*(inputs[REAR_INPUT].getVoltage()/5)),autoscale);
    }

    oscillator.process(args.sampleTime, inputs[SYNK_INPUT].getVoltage());

    // Set output
    if (outputs[OUTPUT].active){
        if(params[LFO_NOISE_PARAM].getValue() == 1){
            outputs[OUTPUT].setVoltage(clamp (5.0f* oscillator.sin() ,-5.0f,5.0f));
        }
        else{
            if(lfo_range == 0){

                outputs[OUTPUT].setVoltage(clamp ((5.0f * oscillator.sin()),-5.0f,5.0f));
            }
            if(lfo_range == 1){
                outputs[OUTPUT].setVoltage(clamp ((5.0f * (1+oscillator.sin())),0.0f,10.0f));
            }

        }


    }


    l_FRONT_PARAM = params[FRONT_PARAM].getValue();
	l_WIDTH_PARAM = params[WIDTH_PARAM].getValue();
	l_REAR_PARAM = params[REAR_PARAM].getValue();
	l_CV_FRONT_PARAM = params[CV_FRONT_PARAM].getValue();
	l_CV_REAR_PARAM = params[CV_REAR_PARAM].getValue();
	l_CV_WIDTH_PARAM = params[CV_WIDTH_PARAM].getValue();
	l_INVERT_PARAM = params[INVERT_PARAM].getValue();
	l_FRONT_INPUT = inputs[FRONT_INPUT].getVoltage();
	l_REAR_INPUT = inputs[REAR_INPUT].getVoltage();
	l_WIDTH_INPUT = inputs[WIDTH_INPUT].getVoltage();
	l_MODE_PARAM = params[MODE_PARAM].getValue();
	l_FM_INPUT = inputs[FM_INPUT].getVoltage();


}

struct OscDisplay : TransparentWidget {
	WCO_Osc *module;
	std::shared_ptr<Font> font;
	OscDisplay() {
		//font = Font::load(assetPlugin(pluginInstance, "res/DejaVuSansMono.ttf"));
	}

    void draw(const DrawArgs &args) override {
        if(module){




            nvgSave(args.vg);
            nvgBeginPath(args.vg);

            nvgRect(args.vg, 0,0, 64,56);
            nvgFillColor(args.vg, nvgRGBA(17,17,17,255));
            nvgFill(args.vg);
            nvgClosePath(args.vg);
            nvgRestore(args.vg);

            nvgSave(args.vg);




            nvgBeginPath(args.vg);
            nvgStrokeColor(args.vg, nvgRGBA(180,50,50,255));
            nvgStrokeWidth(args.vg, 1.5f);
            for(int i=0;i<=64;i++){
                int index = i*4;
                int x = i;
                float y;
                if(i==64)
                    y = this->module->oscillator.buf_wavefront[255];
                else
                    y = this->module->oscillator.buf_wavefront[index];
                y=y*28;
                if(i==0){
                    nvgMoveTo(args.vg, 0, 28-y);
                }
                else{
                    nvgLineTo(args.vg, x, 28-y);
                }
            }
            nvgStroke(args.vg);
            nvgClosePath(args.vg);

            nvgBeginPath(args.vg);
            nvgStrokeColor(args.vg, nvgRGBA(120,120,255,255));
            nvgStrokeWidth(args.vg, 1.5f);
            for(int i=0;i<=64;i++){
                int index = i*4;
                int x = i;
                float y;
                if(this->module->l_INVERT_PARAM != 0){
                    if(i==64)
                        y = this->module->oscillator.buf_waverear[255];
                    else
                        y = this->module->oscillator.buf_waverear[255-index];
                }
                else{
                    if(i==64)
                        y = this->module->oscillator.buf_waverear[255];
                    else
                        y = this->module->oscillator.buf_waverear[index];
                }
                y=y*28;
                if(i==0){
                    nvgMoveTo(args.vg, 0, 28-y);
                }
                else{
                    nvgLineTo(args.vg, x, 28-y);
                }
            }
            nvgStroke(args.vg);
            nvgClosePath(args.vg);

            nvgBeginPath(args.vg);
            nvgStrokeColor(args.vg, nvgRGBA(200,200,200,255));
            nvgStrokeWidth(args.vg, 1.2f);
            //nvgMoveTo(args.vg, 0, 28);
            int al_window = this->module->oscillator.al_window*256;
            int ar_window = this->module->oscillator.ar_window*256;
            int bl_window = this->module->oscillator.bl_window*256;
            int br_window = this->module->oscillator.br_window*256;

            for(int i=0;i<=64;i++){
                int x = i;
                float y;
                int index = i*4;
        //INVERT
                if(this->module->oscillator.invert){
            //SIMPLE
                       if(this->module->oscillator._dual < 0.5f){
                            if( (i > al_window/4 and i < ar_window/4) or ( al_window/4 == 0.0f and ar_window/4 == 64.0f)) {
                                if(i==64){
                                    y = this->module->oscillator.buf_waverear[255];
                                }
                                else{
                                    y = this->module->oscillator.buf_waverear[255-index];
                                }
                            }
                            else{
                                 if(i==64)
                                    y = this->module->oscillator.buf_wavefront[255];
                                else
                                    y = this->module->oscillator.buf_wavefront[index];
                            }
                       }
            //DUAL
                       else{
                            if( (i > al_window/4 and i < ar_window/4) or (i > bl_window/4 and i < br_window/4)or ( al_window/4 == 0.0f and ar_window/4 == 32.0f)) {
                                if(i==64){
                                    y = this->module->oscillator.buf_waverear[255];
                                }
                                else{
                                    y = this->module->oscillator.buf_waverear[255-index];
                                }
                            }
                           else{
                                if(i==64)
                                    y = this->module->oscillator.buf_wavefront[255];
                                else
                                    y = this->module->oscillator.buf_wavefront[index];
                           }
                       }
                    }
        //NORMAL
                    else{
                        if(this->module->oscillator._dual < 0.5f){
                            if( (i > al_window/4 and i < ar_window/4) or ( al_window/4 == 0.0f and ar_window/4 == 64.0f) ) {
                                    if(i==64)
                                        y = this->module->oscillator.buf_waverear[255];
                                    else
                                        y = this->module->oscillator.buf_waverear[index];
                            }
                            else{
                                if(i==64)
                                        y = this->module->oscillator.buf_wavefront[255];
                                else
                                        y = this->module->oscillator.buf_wavefront[index];
                            }
                        }
                        else{
                            if( (i >= al_window/4 and i < ar_window/4) or (i > bl_window/4 and i <= br_window/4)or ( al_window/4 == 0.0f and ar_window/4 == 32.0f)) {
                                if(i==64)
                                    y = this->module->oscillator.buf_waverear[255];
                                else
                                    y = this->module->oscillator.buf_waverear[index];
                            }
                           else{
                                if(i==64)
                                    y = this->module->oscillator.buf_wavefront[255];
                                else
                                    y = this->module->oscillator.buf_wavefront[index];
                           }
                        }
                    }
                y=y*28;
                if(i==0){
                    nvgMoveTo(args.vg, 0, 28-y);
                }
                else{
                    nvgLineTo(args.vg, x, 28-y);
                }
            }
        nvgLineCap(args.vg, NVG_ROUND);
        nvgLineJoin(args.vg, NVG_ROUND);
            nvgStroke(args.vg);
            nvgClosePath(args.vg);


            nvgRestore(args.vg);
        }
        else
            return;

    }
};


struct WCO_OscWidget : ModuleWidget {
    Menu *createContextMenu();
	WCO_OscWidget(WCO_Osc *module);
};

WCO_OscWidget::WCO_OscWidget(WCO_Osc *module) {
		setModule(module);
	setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/WCO_Osc.svg")));
	addChild(createWidget<ScrewSilver>(Vec(15, 0)));
	addChild(createWidget<ScrewSilver>(Vec(box.size.x-30, 0)));
	addChild(createWidget<ScrewSilver>(Vec(15, 365)));
	addChild(createWidget<ScrewSilver>(Vec(box.size.x-30, 365)));

        OscDisplay *display = new OscDisplay();
		display->module = module;
		display->box.pos = Vec(43.0f, 32.0f);
		display->box.size = Vec(110.0f, 68.0f);
		addChild(display);


	addParam(createParam<CKSS>(Vec(15, 48), module, WCO_Osc::MODE_PARAM));
	addParam(createParam<CKSS>(Vec(122, 48), module, WCO_Osc::INVERT_PARAM));
    addParam(createParam<CKSS>(Vec(68.6, 330), module, WCO_Osc::LFO_NOISE_PARAM));


    addParam(createParam<EdgeRedKnob>(Vec(14.8, 211.8), module, WCO_Osc::FRONT_PARAM));
	addParam(createParam<RoundLargeBlackKnob>(Vec(56.5, 187.3), module, WCO_Osc::WIDTH_PARAM));
	addParam(createParam<EdgeBlueKnob>(Vec(108, 211.8), module, WCO_Osc::REAR_PARAM));
	addParam(createParam<RoundSmallBlackKnob>(Vec(17.7, 255), module, WCO_Osc::CV_FRONT_PARAM));
	addParam(createParam<RoundSmallBlackKnob>(Vec(63.5, 248.5), module, WCO_Osc::CV_WIDTH_PARAM));
	addParam(createParam<RoundSmallBlackKnob>(Vec(110.6, 254.8), module, WCO_Osc::CV_REAR_PARAM));
    addParam(createParam<RoundBlackKnob>(Vec(37.5, 101), module, WCO_Osc::FREQ_PARAM));
    addParam(createParam<RoundBlackKnob>(Vec(84.5, 101), module, WCO_Osc::FINE_PARAM));
	addParam(createParam<RoundSmallBlackKnob>(Vec(63.5, 154.1), module, WCO_Osc::FM_PARAM));


	addInput(createInput<PJ301MPort>(Vec(30.5, 154.3), module, WCO_Osc::FM_INPUT));
	addInput(createInput<PJ301MPort>(Vec(95.5, 154.3), module, WCO_Osc::SYNK_INPUT));
	addOutput(createOutput<PJ301MPort>(Vec(110.5, 328), module, WCO_Osc::OUTPUT));
	addInput(createInput<PJ301MPort>(Vec(17.5, 328), module, WCO_Osc::PITCH_INPUT));
    addInput(createInput<PJ301MPort>(Vec(17.5, 300), module, WCO_Osc::FRONT_INPUT));
	addInput(createInput<PJ301MPort>(Vec(63, 300), module, WCO_Osc::WIDTH_INPUT));
	addInput(createInput<PJ301MPort>(Vec(110.5, 300), module, WCO_Osc::REAR_INPUT));
}

/*

struct WCO_OscItem1 : MenuItem {
	WCO_Osc *pt_WCO_Osc;
	void onAction(EventAction &e) override {
	    pt_WCO_Osc->lfo_range = 0;
	}
};

struct WCO_OscItem2 : MenuItem {
	WCO_Osc *pt_WCO_Osc;
	void onAction(EventAction &e) override {
        pt_WCO_Osc->lfo_range = 1;
	}
};

struct WCO_OscItem3 : MenuItem {
	WCO_Osc *pt_WCO_Osc;
	void onAction(EventAction &e) override {
        if(pt_WCO_Osc->autoscale == 1){
            pt_WCO_Osc->autoscale = 0;
        }
        else{
            pt_WCO_Osc->autoscale = 1;
        }
        pt_WCO_Osc->l_CV_FRONT_PARAM = pt_WCO_Osc->l_CV_FRONT_PARAM+0.01;
	}
	void process(const ProcessArgs &args) override {
		rightText = pt_WCO_Osc->autoscale ? "✔" : "";
		MenuItem::step();
	}
};



Menu *WCO_OscWidget::createContextMenu() {
    Menu *menu = ModuleWidget::createContextMenu();

    MenuLabel *spacerLabel = new MenuLabel();
    menu->addChild(spacerLabel);

    WCO_Osc *pt_WCO_Osc = dynamic_cast<WCO_Osc*>(module);
    assert(pt_WCO_Osc);

    WCO_OscItem1 *Range1 = new WCO_OscItem1();
    Range1->text = "LFO Range -5 / 5 V";
    Range1->pt_WCO_Osc = pt_WCO_Osc;
    menu->addChild(Range1);

    WCO_OscItem2 *Range2 = new WCO_OscItem2();
    Range2->text = "LFO Range 0 /  10 V";
    Range2->pt_WCO_Osc = pt_WCO_Osc;
    menu->addChild(Range2);

    WCO_OscItem3 *AutoScale = new WCO_OscItem3();
    AutoScale->text = "Autoscale";
    AutoScale->pt_WCO_Osc = pt_WCO_Osc;
    menu->addChild(AutoScale);

    return menu;
}

*/




Model *modelWCO_Osc = createModel<WCO_Osc, WCO_OscWidget>("WCO_Osc");
