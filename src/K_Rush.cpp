#include "Edge.hpp"

#include "dsp/resampler.hpp"
#include "dsp/filter.hpp"
#include <iostream>
#include <fstream>
#include <string>





using namespace std;


/**Diode -> in  -1V / +1V **/
struct Diode{
    float phase_in, phase_out = 0.0f;
    Upsampler<4,16> Upsample;
    Decimator<4,16> Decimate;
    float Ov_Buffer[16] = {0};

    RCFilter filter1;

    string plug_directory = assetPlugin(plugin, "res/waves2/");
	float wave[64][256]={{0}};
    const string wavefiles[64]={"00.wav","01.wav","02.wav","03.wav","04.wav","05.wav","06.wav","07.wav","08.wav","09.wav","10.wav","11.wav","12.wav","13.wav","14.wav","15.wav","16.wav","17.wav","18.wav","19.wav","20.wav","21.wav","22.wav","23.wav","24.wav","25.wav","26.wav","27.wav","28.wav","29.wav","30.wav","31.wav","32.wav","33.wav","34.wav","35.wav","36.wav","37.wav","38.wav","39.wav","40.wav","41.wav","42.wav","43.wav","44.wav","45.wav","46.wav","47.wav","48.wav","49.wav","50.wav","51.wav","52.wav","53.wav","54.wav","55.wav","56.wav","57.wav","58.wav","59.wav","60.wav","61.wav","62.wav","63.wav"};
    FILE *wave_f = NULL;
	short temp_buf[256]={0};
    bool tab_loaded = false;
    float out = 0.0f;



    void LoadWaves(){

        for(int j=0; j<64; j++){
            string file_name = plug_directory+wavefiles[j];
            const char *c = file_name.c_str();
            wave_f = NULL;
            wave_f = fopen(c,"r");
            if(wave_f!=NULL){
                fseek(wave_f,44,SEEK_SET);
                fread(temp_buf,sizeof(temp_buf),256,wave_f);
                for(int i = 0; i<256 ; i++){
                    wave[j][i] = ((float)temp_buf[i])/pow(256,2);
                }
                fclose(wave_f);
            }
            else{
                j=0;

            }
        }
        tab_loaded = true;
    }



    float proc_f_d1(float in, float gain, int type,float feedback){

        if(tab_loaded == false){
            LoadWaves();
        }


        float last_dif = clamp((out-in)/16.0f,0.0f,1.0f);
        in=(in-(feedback*(gain/8.0f)));
        Upsample.process(in,Ov_Buffer);
        filter1.setCutoff((44100 * (engineGetSampleTime()/16)));
        float index = 0.0f;


        //OVRS
        for(int i = 0 ; i< 4 ; i++){

            if (Ov_Buffer[i]<0)
                phase_in = -1.0f;
            else
                phase_in = 1.0f;


            float abs_in = abs(Ov_Buffer[i]);


            index =  tanh(abs_in)*255;
            //clamp(index,0.0f,255.0f);


            clamp(index,0.0f,255.0f);


            abs_in *= 1- (clamp((gain-1),0.0f,8.0f)*interpolateLinear(wave[type],index)+0.5f);



            abs_in *= gain;

            if(phase_in>0.0f)
                Ov_Buffer[i] = phase_in*abs_in;
            else
                Ov_Buffer[i] = phase_in*(abs_in);

            filter1.process(Ov_Buffer[i]);
            Ov_Buffer[i]=filter1.lowpass();

        }
        in=Decimate.process(Ov_Buffer)*2.0f;//*phase_in;
        out = clamp(in,-1.0f,1.0f);
        return out ;


    }


};



struct K_Rush : Module {
	enum ParamIds {
	    TRIM_PARAM,
	    MIX_PARAM,
		FEEDBACK_PARAM,
		GAIN_PARAM,
		WAVET_PARAM,
		CV_GAIN_PARAM,
		CV_FEEDBACK_PARAM,
		NUM_PARAMS
	};
	enum InputIds {
		CV_GAIN_INPUT,
		IN_INPUT,
		CV_FEEDBACK_INPUT,
		FEEDBACK_INPUT,
		NUM_INPUTS
	};
	enum OutputIds {
		OUT_OUTPUT,
		FEEDBACK_OUTPUT,
		NUM_OUTPUTS
	};
	enum LightIds {
		NUM_LIGHTS
	};


    float feed_back = 0.0f;


	/*****************

    const float m_thermalVoltage = 0.025864f;
	const float m_saturationCurrent = 10e-17f;
	float m_resistor = 15000.f;
	float m_loadResistor = 7500.f;
	float m_loadResistor2 = m_loadResistor * 2.f;
	// Derived values
	float m_alpha = m_loadResistor2 / m_resistor;
	float m_beta = (m_resistor + m_loadResistor2) / (m_thermalVoltage * m_resistor);
	float m_delta = (m_loadResistor * m_saturationCurrent) / m_thermalVoltage;

	*******************/

    Diode d_pos,d_neg;

	K_Rush() : Module(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS) {
	}



	void step() override;

	// For more advanced Module features, read Rack's engine.hpp header file
	// - toJson, fromJson: serialization of internal data
	// - onSampleRateChange: event triggered by a change of sample rate
	// - onReset, onRandomize, onCreate, onDelete: implements special behavior when user clicks these from the context menu
};





void K_Rush::step() {


    float gain = params[GAIN_PARAM].value;


    float feed_back = clamp((params[FEEDBACK_PARAM].value + (params[CV_FEEDBACK_PARAM].value*inputs[CV_FEEDBACK_INPUT].value)),0.0f,1.0f)*(inputs[FEEDBACK_INPUT].value/5.0f);
    //float in = (inputs[IN_INPUT].value/5.0f)-((inputs[FEEDBACK_INPUT].value/5.0f)/clamp(16/gain,1.0f,16.0f) *feed_back);
    float in = (inputs[IN_INPUT].value/5.0)*params[TRIM_PARAM].value;
    if(inputs[CV_GAIN_INPUT].active)
        gain += (inputs[CV_GAIN_INPUT].value*params[CV_GAIN_PARAM].value);

    gain = clamp(gain,0.0f,8.0f);
    int type_diode = params[WAVET_PARAM].value;
    in = d_pos.proc_f_d1(in ,gain,type_diode,feed_back);
    outputs[OUT_OUTPUT].value = (params[MIX_PARAM].value*in*5)+((1-params[MIX_PARAM].value)*inputs[IN_INPUT].value);
    outputs[FEEDBACK_OUTPUT].value = in*5;


}


struct K_RushWidget : ModuleWidget {
	K_RushWidget(K_Rush *module) : ModuleWidget(module) {
		setPanel(SVG::load(assetPlugin(plugin, "res/K_Rush.svg")));

		addParam(ParamWidget::create<RoundSmallBlackKnob>(Vec(15.2, 85.5), module, K_Rush::TRIM_PARAM, -4.0f, 4.0f, 1.0f));
        addParam(ParamWidget::create<RoundBlackSnapKnob>(Vec(60.5, 82.8), module, K_Rush::WAVET_PARAM, 0.0f, 15.0f, 0.0f));
        addParam(ParamWidget::create<RoundSmallBlackKnob>(Vec(110.9, 85.5), module, K_Rush::MIX_PARAM, 0.0f, 1.0f, 1.0f));

		addParam(ParamWidget::create<RoundLargeBlackKnob>(Vec(12.2, 158.7), module, K_Rush::GAIN_PARAM, 0.0f, 8.0f, 1.0f));
		addParam(ParamWidget::create<RoundLargeBlackKnob>(Vec(100, 256.7), module, K_Rush::FEEDBACK_PARAM, 0.0f, 1.0f, 0.0f));

		addParam(ParamWidget::create<RoundSmallBlackKnob>(Vec(106.9, 165.7), module, K_Rush::CV_GAIN_PARAM, -1.0f, 1.0f, 0.0f));
		addParam(ParamWidget::create<RoundSmallBlackKnob>(Vec(19.3, 263.8), module, K_Rush::CV_FEEDBACK_PARAM, 0.0f, 0.3f, 0.0f));

		addInput(Port::create<PJ301MPort>(Vec(62.3, 205), Port::INPUT, module, K_Rush::CV_GAIN_INPUT));
        addInput(Port::create<PJ301MPort>(Vec(62.3, 302.6), Port::INPUT, module, K_Rush::CV_FEEDBACK_INPUT));

        addInput(Port::create<PJ301MPort>(Vec(9.3, 345), Port::INPUT, module, K_Rush::FEEDBACK_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(62.3, 345), Port::INPUT, module, K_Rush::IN_INPUT));
		addOutput(Port::create<PJ301MPort>(Vec(115.3, 345), Port::OUTPUT, module, K_Rush::OUT_OUTPUT));


	}
};


// Specify the Module and ModuleWidget subclass, human-readable
// author name for categorization per plugin, module slug (should never
// change), human-readable module name, and any number of tags
// (found in `include/tags.hpp`) separated by commas.
Model *modelK_Rush = Model::create<K_Rush, K_RushWidget>("Edge", "K_Rush", "K_Rush", WAVESHAPER_TAG);



