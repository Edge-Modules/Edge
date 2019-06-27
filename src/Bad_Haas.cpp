#include "Edge.hpp"



struct Bad_Haas : Module {
	enum ParamIds {
	    DELAY_LR,
	    CV_DELAY_LR,
	    DRY_WET,
	    CV_DRY_WET,
        NUM_PARAMS
	};
	enum InputIds {
	    INPUT,
	    IN_Delay_LR,
	    IN_DRY_WET,
        NUM_INPUTS
	};
	enum OutputIds {
	    OUT_L,
	    OUT_R,
        NUM_OUTPUTS
	};
	enum LightIds {
        NUM_LIGHTS
	};

	int sample_rate = APP->engine->getSampleRate();

    float Delay_buff[2048]={0};;
    float last_panningLR,last_panningUB = 0.0f;
    float last_index_l, last_index_r = 0.0f;
    int index = 0;
    dsp::RCFilter filter1;
	Bad_Haas() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
		configParam(DELAY_LR, -1.0f, 1.0f, 0.0f);
		configParam(CV_DELAY_LR, -1.0f, 1.0f, 0.0f);
		configParam(DRY_WET, -1.0f, 1.0f, 0.0f);
		configParam(CV_DRY_WET, -1.0f, 1.0f, 0.0f);
	}
	void process(const ProcessArgs &args) override;
	float smp_to_ms(float smp);
	float ms_to_smp(float ms);



};

float fade(float in, float last_in){
    float dif = 0;
    dif = in - last_in;
    if(dif>0){
        in += std::min(dif,0.0001f);
    }
    if(dif<0){
        in += std::max(dif,-0.0001f);
    }
    return in;
}

float Bad_Haas::smp_to_ms(float smp){
    return smp*1000/sample_rate;
}

float Bad_Haas::ms_to_smp(float ms){
    return ms*sample_rate/1000;
}

void Bad_Haas::process(const ProcessArgs &args)  {

    float dry_wet = clamp(params[DRY_WET].getValue()+((inputs[IN_DRY_WET].getVoltage()/5.0f)*params[CV_DRY_WET].getValue()),-1.0f,1.0f);

    if(index==2048){index=0;}
    Delay_buff[index]= inputs[INPUT].getVoltage() ;

    // LEFT / RIGHT PART
    float panningLR = clamp(params[DELAY_LR].getValue()+((inputs[CV_DELAY_LR].getVoltage()/5.0f)*params[CV_DELAY_LR].getValue()),-1.0f,1.0f);

    filter1.setCutoff(50 * APP->engine->getSampleTime());
    filter1.process(panningLR);
    panningLR=filter1.lowpass();

    float coefL, coefR = 0.0f;
    coefL = clamp(panningLR,0.0f,1.0f);
    coefR = abs(clamp(panningLR,-1.0f,0.0f));

    float indexL = index - (ms_to_smp(2)*coefL);
    float indexR = index - (ms_to_smp(2)*coefR);
    float temp_l =  indexL;
    if(temp_l<=0){temp_l+=2047;}
    float temp_r =  indexR;
    if(temp_r<=0){temp_r+=2047;}

    //test early reflection
/*
    float indexL2 = indexL - (ms_to_smp(1)*coefL);
    float indexR2 = indexR - (ms_to_smp(1)*coefR);

    float temp_l2 =  indexL2;
    if(temp_l2<=0){temp_l2+=2047;}
    float temp_r2 =  indexR2;
    if(temp_r2<=0){temp_r2+=2047;}

    //

*/

    float dry = (2-(dry_wet+1))/2.0f;
    float wet = (dry_wet+1)/2.0f;

    float out__l = dry*inputs[INPUT].getVoltage() + wet*interpolateLinear(Delay_buff,temp_l);//+(interpolateLinear(Delay_buff,temp_l2)*0.4f);
    float out__r = dry*inputs[INPUT].getVoltage() + wet*interpolateLinear(Delay_buff,temp_r);//+(interpolateLinear(Delay_buff,temp_r2)*0.4f);

    // ---------------------------------------------



    // ---------------------------------------------

    outputs[OUT_R].setVoltage(out__r);
    outputs[OUT_L].setVoltage(out__l);
    last_panningLR = panningLR;
    last_index_l = indexL;
    last_index_r = indexR;
    //last_panningUB = panningUB;
    index++;
}


struct Bad_HaasWidget : ModuleWidget {
	Bad_HaasWidget(Bad_Haas *module);
};

Bad_HaasWidget::Bad_HaasWidget(Bad_Haas *module) {
		setModule(module);
	setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/Bad_Haas.svg")));


    addParam(createParam<RoundBlackKnob>(Vec(7.74, 67.7), module, Bad_Haas::DELAY_LR));
    addParam(createParam<RoundSmallBlackKnob>(Vec(10.7, 107.7), module, Bad_Haas::CV_DELAY_LR));
    addParam(createParam<RoundBlackKnob>(Vec(7.74, 180.5), module, Bad_Haas::DRY_WET));
    addParam(createParam<RoundSmallBlackKnob>(Vec(10.7, 220.5), module, Bad_Haas::CV_DRY_WET));



    addInput(createInput<PJ301MPort>(Vec(9.7, 136.7),  module, Bad_Haas::IN_Delay_LR));
    addInput(createInput<PJ301MPort>(Vec(9.7, 247.7),  module, Bad_Haas::IN_DRY_WET));

	addInput(createInput<PJ301MPort>(Vec(9.7, 282),  module, Bad_Haas::INPUT));
    addOutput(createOutput<PJ301MPort>(Vec(9.7, 315),  module, Bad_Haas::OUT_L));
    addOutput(createOutput<PJ301MPort>(Vec(9.7, 338),  module, Bad_Haas::OUT_R));

}


Model *modelBad_Haas = createModel<Bad_Haas, Bad_HaasWidget>("Bad_Haas");

