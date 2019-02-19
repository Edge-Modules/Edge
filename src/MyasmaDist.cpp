#include "Edge.hpp"
//#include "Dep/IIR/IIRDecimator.h"
//#include "Dep/IIR/IIRUpsampler.h"
#include "Dep/LambertW.hpp"
#include "dsp/resampler.hpp"
#include "dsp/filter.hpp"
#include <iostream>
#include <fstream>
#include <string>



using namespace std;


inline float clip(float x) {
	return tanhf(x);
}

/**Diode -> in  -1V / +1V **/
struct Diode{
    float offset,limit1,limit2,phase_in, phase_out = 0.0f;
    float weight_accum = 0;
    Upsampler<16,16> Upsample;
    Decimator<16,16> Decimate;
    float Ov_Buffer[16] = {0};

    RCFilter filter1;

    string plug_directory = assetPlugin(plugin, "res/waves2/");
	float wave[64][256]={{0}};
    const string wavefiles[64]={"00.wav","01.wav","02.wav","03.wav","04.wav","05.wav","06.wav","07.wav","08.wav","09.wav","10.wav","11.wav","12.wav","13.wav","14.wav","15.wav","16.wav","17.wav","18.wav","19.wav","20.wav","21.wav","22.wav","23.wav","24.wav","25.wav","26.wav","27.wav","28.wav","29.wav","30.wav","31.wav","32.wav","33.wav","34.wav","35.wav","36.wav","37.wav","38.wav","39.wav","40.wav","41.wav","42.wav","43.wav","44.wav","45.wav","46.wav","47.wav","48.wav","49.wav","50.wav","51.wav","52.wav","53.wav","54.wav","55.wav","56.wav","57.wav","58.wav","59.wav","60.wav","61.wav","62.wav","63.wav"};
    FILE * wave_f = NULL;
	short temp_buf[256]={0};
    bool tab_loaded = false;


    void LoadWaves(){

        for(int j=0; j<64; j++){
            float chkcnt = 0;
            string file_name = plug_directory+wavefiles[j];
            const char *c = file_name.c_str();
            wave_f = fopen(c,"r");
            if(wave_f!=NULL){
                fseek(wave_f,44,SEEK_SET);
                fread(temp_buf,sizeof(temp_buf),256,wave_f);
                for(int i = 0; i<256 ; i++){
                    if(temp_buf[i]!=0.0f){
                        chkcnt++;
                    }
                    wave[j][i] = ((float)temp_buf[i]/pow(256,2)+0.5);

                }
                if(chkcnt == 0){
                tab_loaded = false;
                j=64;
                }
                fclose(wave_f);
            }
            else{
                tab_loaded = false;
            }
            if(j==63){
                tab_loaded=true;
            }
        }
         //_fcloseall();

    }



    float proc_f_d1(float in, float gain, int type,float feedback){

        if(tab_loaded == false){
            LoadWaves();
        }


       // in *= gain;
        //float weight = 0.0f;


        //weight = abs(d_memory[0] - d_memory[1]);




        //phase_out = phase_in;
       // weight_accum += weight* phase_in;
/*
        if(weight_accum > 1.0f){
            weight_accum = d_memory[0]-weight*-1;

        }
        if(weight_accum < -1.0f){
            weight_accum = d_memory[0]+weight*-1;
        }


*/

        //weight_accum = clamp(weight_accum,-1.0f,1.0f);




        //weight = clamp( weight, -0.1, 0.1);

       //  float out = weight_accum;

       // float out = d_memory[1] + (weight*phase);

/*
        float phase,weight = 0.0f;
        d_memory[1] = d_memory[0];
        d_memory[0] = in;

        if(d_memory[0] > d_memory[1]){
            phase = 1.0f;
            weight = d_memory[0] - d_memory[1];
        }
        else{
                if(d_memory[0] == d_memory[1]){
                    phase = 0.0f;
                    weight = 0.0f;
                }
                else{
                    phase = -1.0f;
                    weight = d_memory[1] - d_memory[0];
                }
        }
*/
/*
        if(weight > 0.008)
            phase = -phase;
*/
        //float temp=in;

        /**-> 0 à +1 **/

        //if(in<0)
          //  in=in*-1;
        //in = -1*pow(in*gain,2)+1;


       /**WAVEFOLD**/
        //if(in >= 1.0f)
        //    in = 1+(1-in);
        //if(in <= -1.0f)
        //    in = -1-(in+1);


        //in = sgn(temp)*in;

        //float out = (d_memory[1]+weight)*phase;


        //out=tanh(gain*out);

        Upsample.process(in,Ov_Buffer);
        filter1.setCutoff((44100 *engineGetSampleTime()/16));
        float index= 0.0f;

        for(int i = 0 ; i< 16 ; i++){

            if(Ov_Buffer[i]>=0)
                phase_in=1.0f;
            if(Ov_Buffer[i]<0)
                phase_in= -1.0f;
            Ov_Buffer[i] = abs(gain * Ov_Buffer[i]);
            index =  clamp(Ov_Buffer[i]-1.0f, 0.0f, 20.0f)*64;
            while(index>255){
                index-=255;
            }
            while(index<0){
                index+=255;
            }
            Ov_Buffer[i] =  Ov_Buffer[i]-((gain/2-1.0f) * interpolateLinear(wave[type],index));

            Ov_Buffer[i] = Ov_Buffer[i]*phase_in;

            filter1.process(Ov_Buffer[i]);
            Ov_Buffer[i]=filter1.lowpass();
        }
        in=Decimate.process(Ov_Buffer);
        float out = clamp(in,-1.0f,1.0f);
        return out ;


    }


};



struct MyasmaDist : Module {
	enum ParamIds {
		FEEDBACK_PARAM,
		GAIN_PARAM,
		BLEND_PARAM,
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

	MyasmaDist() : Module(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS) {
	}



	void step() override;

	// For more advanced Module features, read Rack's engine.hpp header file
	// - toJson, fromJson: serialization of internal data
	// - onSampleRateChange: event triggered by a change of sample rate
	// - onReset, onRandomize, onCreate, onDelete: implements special behavior when user clicks these from the context menu
};





void MyasmaDist::step() {

    feed_back = params[FEEDBACK_PARAM].value + (params[CV_FEEDBACK_PARAM].value*inputs[CV_FEEDBACK_INPUT].value);
    feed_back = clamp(feed_back,0.0f,1.0f);
    float in = (inputs[IN_INPUT].value/5.0f)-(inputs[FEEDBACK_INPUT].value/5.0f*feed_back);
    float gain = params[GAIN_PARAM].value;

    if(inputs[CV_GAIN_INPUT].active)
        gain += (inputs[CV_GAIN_INPUT].value*params[CV_GAIN_PARAM].value);

    gain = clamp(gain,0.0f,16.0f);


    /*****WAvefolder chelou (1 seule rebond en haut / infini en bas

    if(in >= 1.0f)
        in = 1+(1-in);
    if(in <= -1.0f)
        in = -1-(in+1);
    ***/

    /***HARDCLIPPER
    if(in >= 1.0f)
        in+=(1-in);
    if(in <= -1.0f)
        in-=(1+in);
    *****/

    //Upsample.process(in,Ov_Buffer);

    /**** SQUAREWAVE Le signal;
    in = sgn(in)*(in+(1-in));
    ****/

    //d_pos.offset=-2.0f;
    /***   Offset et HardClip    ***/

    //in = in+d_pos.offset+d_neg.offset;




/***Fonction Diode+ Diode- */

    //in=gain*in;


   // in = d_pos.proc_f_d1(in ,gain,1);


    /*

    filter1.setCutoff(40 / (engineGetSampleTime()/16));
    for(int i = 0 ; i< 16 ; i++){
        Ov_Buffer[i] = d_pos.proc_f_d1(Ov_Buffer[i] ,gain,1);
        filter1.process(Ov_Buffer[i]);
        Ov_Buffer[i]=filter1.lowpass();
    }


    // DECIMATION !
     in=Decimate.process(Ov_Buffer);


    //in = Decimate.process(Ov_Buffer);
    //in = clamp(in,-5.0f,5.0f);
*/
    int type_diode = int(params[BLEND_PARAM].value *16);
    in = d_pos.proc_f_d1(in ,gain,type_diode,params[FEEDBACK_PARAM].value);
    outputs[OUT_OUTPUT].value = in*5;
    outputs[FEEDBACK_OUTPUT].value = in*5;
/*
    //float out = in;

    //outputs[OUT_OUTPUT].value = out;


// DIODE

    float result = 0.0f;
    float feedback = params[FEEDBACK_PARAM].value;


    //float in =inputs[IN_INPUT].value/5.0f;
    buff[0]=in;
    for(int i = 905 ; i >= 0 ; i--){
        buff[i+1]=buff[i];
    }

    if(inputs[FEEDBACK_INPUT].active){
        in += feedback*inputs[FEEDBACK_INPUT].value/5;
    }
    else{
        in += feedback*buff[280]/2;
    }
    in*=gain;
    in = tanh(in);
    const float theta = sgn(in);

    Upsample.process(in,Ov_Buffer);

    for (int i = 0; i < 2; i++) {
        Ov_Buffer[i] =  m_alpha*Ov_Buffer[i] - (theta * m_thermalVoltage * LambertW(m_delta * exp(theta * m_beta * Ov_Buffer[i])));
    }
    result = Decimate.process(Ov_Buffer);
    result = m_alpha*in - (theta * m_thermalVoltage * LambertW(m_delta * exp(theta * m_beta * in))) ;
    outputs[OUT_OUTPUT].value = tanh(result)*20;
    outputs[FEEDBACK_OUTPUT].value = tanh(result)*20;

    last_step = tanh(result);

*///////////////////////////////////////


}


struct MyasmaDistWidget : ModuleWidget {
	MyasmaDistWidget(MyasmaDist *module) : ModuleWidget(module) {
		setPanel(SVG::load(assetPlugin(plugin, "res/MyasmaDist.svg")));

		addChild(Widget::create<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(Widget::create<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		addChild(Widget::create<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(Widget::create<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addParam(ParamWidget::create<RoundHugeBlackKnob>(Vec(33, 61), module, MyasmaDist::FEEDBACK_PARAM, 0.0f, 1.0f, 0.0f));
		addParam(ParamWidget::create<RoundLargeBlackKnob>(Vec(12, 143), module, MyasmaDist::GAIN_PARAM, 0.0f, 8.0f, 1.0f));
		addParam(ParamWidget::create<RoundLargeBlackKnob>(Vec(71, 143), module, MyasmaDist::BLEND_PARAM, 0.0f, 1.0f, 0.0f));
		addParam(ParamWidget::create<RoundLargeBlackKnob>(Vec(12, 208), module, MyasmaDist::CV_GAIN_PARAM, -1.0f, 1.0f, 0.0f));
		addParam(ParamWidget::create<RoundLargeBlackKnob>(Vec(71, 208), module, MyasmaDist::CV_FEEDBACK_PARAM, 0.0f, 0.3f, 0.0f));

		addInput(Port::create<PJ301MPort>(Vec(10, 276), Port::INPUT, module, MyasmaDist::CV_GAIN_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(48, 276), Port::INPUT, module, MyasmaDist::IN_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(85, 276), Port::INPUT, module, MyasmaDist::CV_FEEDBACK_INPUT));
		addInput(Port::create<PJ301MPort>(Vec(10, 320), Port::INPUT, module, MyasmaDist::FEEDBACK_INPUT));

		addOutput(Port::create<PJ301MPort>(Vec(48, 320), Port::OUTPUT, module, MyasmaDist::OUT_OUTPUT));
		addOutput(Port::create<PJ301MPort>(Vec(85, 320), Port::OUTPUT, module, MyasmaDist::FEEDBACK_OUTPUT));

	}
};


// Specify the Module and ModuleWidget subclass, human-readable
// author name for categorization per plugin, module slug (should never
// change), human-readable module name, and any number of tags
// (found in `include/tags.hpp`) separated by commas.
Model *modelMyasmaDist = Model::create<MyasmaDist, MyasmaDistWidget>("Edge", "MyasmaDist", "MyasmaDist", OSCILLATOR_TAG);



