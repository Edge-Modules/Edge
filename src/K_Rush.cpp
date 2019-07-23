#include "Edge.hpp"


#include "dep/dr_wav.h"




using namespace std;
static const size_t BLOCK_SIZE = 512;
static const float PI = 3.1416f;



/**Diode -> in  -1V / +1V **/
struct Diode{
    float phase_in, phase_out = 0.0f;
    dsp::Upsampler<4,16> Upsample;
    dsp::Decimator<4,16> Decimate;
    float Ov_Buffer[16] = {0};

    dsp::RCFilter filter1;

    std::string plug_directory = asset::plugin(pluginInstance, "res/waves2/");
	float wave[64][256]={{0.0f}};
    const std::string wavefiles[64]={"00.wav","01.wav","02.wav","03.wav","04.wav","05.wav","06.wav","07.wav","08.wav","09.wav","10.wav","11.wav","12.wav","13.wav","14.wav","15.wav","16.wav","17.wav","18.wav","19.wav","20.wav","21.wav","22.wav","23.wav","24.wav","25.wav","26.wav","27.wav","28.wav","29.wav","30.wav","31.wav","32.wav","33.wav","34.wav","35.wav","36.wav","37.wav","38.wav","39.wav","40.wav","41.wav","42.wav","43.wav","44.wav","45.wav","46.wav","47.wav","48.wav","49.wav","50.wav","51.wav","52.wav","53.wav","54.wav","55.wav","56.wav","57.wav","58.wav","59.wav","60.wav","61.wav","62.wav","63.wav"};

	short temp_buf[256]={0};
    bool tab_loaded = false;
    float out = 0.0f;
    float faded_type = 0.0f;
    bool first_alg = true;
    float l_type = 0;
    float buf_in[BLOCK_SIZE]={0.0f};
    float buf_out[BLOCK_SIZE]={0.0f};
    dsp::RealTimeConvolver *convolver = NULL;

    Diode(){
        LoadWaves();
    }

    ~Diode(){


    }


    void LoadWaves(){

        for(int j=0; j<64; j++){
            std::string file_name = plug_directory+wavefiles[j];
            const char *chemin = file_name.c_str();
            unsigned int channels;
            unsigned int sampleRate;
            drwav_uint64 totalPCMFrameCount;
            float* pSampleData = drwav_open_file_and_read_pcm_frames_f32(chemin, &channels, &sampleRate, &totalPCMFrameCount);
            for(int i = 0; i<256 ; i++){
                wave[j][i] = pSampleData[i]/2.0f;
            }
            drwav_free(pSampleData);;
        }
        tab_loaded = true;
    }



    float proc_f_d1(float in, float gain, float type,float feedback){


        if(!first_alg){
            //gain = (gain+1);


            if(tab_loaded == false){
                LoadWaves();
            }
/*
            in=(in-(feedback*(gain/8.0f)));
            Upsample.process(in,Ov_Buffer);
            filter1.setCutoff((44100 *engineGetSampleTime()/4));
            float index= 0.0f;

            for(int i = 0 ; i< 4 ; i++){

                if(Ov_Buffer[i]>=0)
                    phase_in=1.0f;
                if(Ov_Buffer[i]<0)
                    phase_in= -1.0f;
                Ov_Buffer[i] = abs(Ov_Buffer[i]*gain);
                index =  Ov_Buffer[i]*16;

                while(index>255){
                    index-=255;
                }
                while(index<0){
                    index+=255;
                }
                Ov_Buffer[i] *= 1- ((clamp((gain-1),0.0f,8.0f)*0.2)*(interpolateLinear(wave[(int)type],index)+0.5f));

                Ov_Buffer[i] = Ov_Buffer[i]*phase_in;

                filter1.process(Ov_Buffer[i]);
                Ov_Buffer[i]=filter1.lowpass();
            }
            in=Decimate.process(Ov_Buffer);
            */
            float sin_in = sin(PI*in);
            int index = in*255;
            float cos_wave = cos(PI*(gain*wave[(int)type][index]));
            out = sin_in * cos_wave;

            //float out = clamp(out,-1.0f,1.0f);

            return out ;
        }

        else{
           if(tab_loaded == false){
                LoadWaves();
            }

            float dif_type = type - faded_type;
            if(dif_type>0){
                type += std::min(dif_type,0.01f);
            }
            if(dif_type<0){
                type += std::max(dif_type,-0.01f);
            }


            in=(in-(feedback*(gain/8.0f)));
            Upsample.process(in,Ov_Buffer);
            filter1.setCutoff((44100 * (APP->engine->getSampleTime())));
            float index = 0.0f;

            type = clamp(type, 0.00f,14.99f);

            //OVRS
            for(int i = 0 ; i< 4 ; i++){

                if (Ov_Buffer[i]<0)
                    phase_in = -1.0f;
                else
                    phase_in = 1.0f;


                Ov_Buffer[i] = abs(Ov_Buffer[i]);


                index =  tanh(Ov_Buffer[i])*255;//*clamp(((gain/4.0f)-1.0f),0.0f,2.0f);
                index= 255.0f-index*2;

                index = abs(index);


                clamp(index,0.01f,255.0f);


                float coef1f, coef2f = 0.0f;
                float interp_l = 0.0f;
                if( (int)type > type ){
                    coef1f = (int) type - type;
                    coef2f = 1 - coef1f;
                }
                else{
                    coef2f = type - (int) type;
                    coef1f = 1 - coef2f;
                }
                interp_l+= interpolateLinear(wave[(int)type],index)*coef1f;
                interp_l+= interpolateLinear(wave[(int)type+1],index)*coef2f;

                Ov_Buffer[i] *= 1- ((clamp((gain-1),0.0f,8.0f)*0.2)*(interp_l+0.5f));


                Ov_Buffer[i] *= (gain*2.0f);
                //Ov_Buffer[i] = tanh(Ov_Buffer[i]);
                if(phase_in>0.0f)
                    Ov_Buffer[i] = phase_in*Ov_Buffer[i];
                else
                    Ov_Buffer[i] = phase_in*(Ov_Buffer[i]);

                filter1.process(Ov_Buffer[i]);
                Ov_Buffer[i]=filter1.lowpass();

            }
            in=Decimate.process(Ov_Buffer);//*phase_in;
            out = tanh(in);


            return out ;


        }



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
	    TYPE_INPUT,
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

	K_Rush() {
		config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);
		configParam(TRIM_PARAM, -4.0f, 4.0f, 1.0f);
		configParam(WAVET_PARAM, 0.0f, 15.0f, 0.0f);
		configParam(MIX_PARAM, 0.0f, 1.0f, 1.0f);
		configParam(GAIN_PARAM, 0.0f, 8.0f, 1.0f);
		configParam(FEEDBACK_PARAM, 0.0f, 1.0f, 0.0f);
		configParam(CV_GAIN_PARAM, -1.0f, 1.0f, 0.0f);
		configParam(CV_FEEDBACK_PARAM, 0.0f, 0.3f, 0.0f);
	}



	void process(const ProcessArgs &args) override;

	// For more advanced Module features, read Rack's engine.hpp header file
	// - toJson, fromJson: serialization of internal data
	// - onSampleRateChange: event triggered by a change of sample rate
	// - onReset, onRandomize, onCreate, onDelete: implements special behavior when user clicks these from the context menu

    json_t *dataToJson() override {
		json_t *rootJ = json_object();
		json_object_set_new(rootJ, "first_alg", json_integer(d_pos.first_alg));
		return rootJ;
	}

	void dataFromJson(json_t *rootJ) override {
		json_t *first_algJ = json_object_get(rootJ, "first_alg");
		if (d_pos.first_alg)
			d_pos.first_alg = json_integer_value(first_algJ);
	}



};





void K_Rush::process(const ProcessArgs &args) {


    float gain = params[GAIN_PARAM].getValue();


    float feed_back = clamp((params[FEEDBACK_PARAM].getValue() + (params[CV_FEEDBACK_PARAM].getValue()*inputs[CV_FEEDBACK_INPUT].getVoltageSum())),0.0f,1.0f)*(inputs[FEEDBACK_INPUT].getVoltage()/5.0f);
    //float in = (inputs[IN_INPUT].value/5.0f)-((inputs[FEEDBACK_INPUT].value/5.0f)/clamp(16/gain,1.0f,16.0f) *feed_back);
    float in = (inputs[IN_INPUT].getVoltageSum()/5.0)*params[TRIM_PARAM].getValue();
    if(inputs[CV_GAIN_INPUT].active)
        gain += (inputs[CV_GAIN_INPUT].getVoltageSum()*params[CV_GAIN_PARAM].getValue());

    gain = clamp(gain,0.0f,8.0f);
    float type_diode = params[WAVET_PARAM].getValue()+(inputs[TYPE_INPUT].getVoltage() * 2.0f);
    in = d_pos.proc_f_d1(in ,gain,type_diode,feed_back);
    outputs[OUT_OUTPUT].setVoltage((params[MIX_PARAM].getValue()*in*5)+((1-params[MIX_PARAM].getValue())*inputs[IN_INPUT].getVoltageSum()));
    outputs[FEEDBACK_OUTPUT].setVoltage(in*5);


}







struct K_RushWidget : ModuleWidget {
    //Menu *createContextMenu();
	K_RushWidget(K_Rush *module){
        setModule(module);
        setPanel(APP->window->loadSvg(asset::plugin(pluginInstance, "res/K_Rush.svg")));

        //addParam(createParamWidget<RoundSmallBlackKnob>(Vec(15.2, 85.5), module, K_Rush::TRIM_PARAM, -4.0f, 4.0f, 1.0f));
        addParam(createParam<RoundSmallBlackKnob>(Vec(15.2, 85.5), module, K_Rush::TRIM_PARAM));

        addParam(createParam<RoundBlackKnob>(Vec(60.5, 82.8), module, K_Rush::WAVET_PARAM));
        addParam(createParam<RoundSmallBlackKnob>(Vec(110.9, 85.5), module, K_Rush::MIX_PARAM));

        addParam(createParam<RoundLargeBlackKnob>(Vec(12.2, 158.7), module, K_Rush::GAIN_PARAM));
        addParam(createParam<RoundLargeBlackKnob>(Vec(100, 256.7), module, K_Rush::FEEDBACK_PARAM));

        addParam(createParam<RoundSmallBlackKnob>(Vec(106.9, 165.7), module, K_Rush::CV_GAIN_PARAM));
        addParam(createParam<RoundSmallBlackKnob>(Vec(19.3, 263.8), module, K_Rush::CV_FEEDBACK_PARAM));

        addInput(createInput<PJ301MPort>(Vec(62.3, 125),  module, K_Rush::TYPE_INPUT));
        addInput(createInput<PJ301MPort>(Vec(62.3, 205),  module, K_Rush::CV_GAIN_INPUT));
        addInput(createInput<PJ301MPort>(Vec(62.3, 302.6), module, K_Rush::CV_FEEDBACK_INPUT));

        addInput(createInput<PJ301MPort>(Vec(9.3, 345), module, K_Rush::FEEDBACK_INPUT));
        addInput(createInput<PJ301MPort>(Vec(62.3, 345), module, K_Rush::IN_INPUT));
        addOutput(createOutput<PJ301MPort>(Vec(115.3, 345), module, K_Rush::OUT_OUTPUT));


        }


        struct AlgoSelecItem : MenuItem {
            K_Rush *pt_kr;
            void onAction(const event::Action &e) override {

                    pt_kr->d_pos.first_alg = true;
            }
            void step() override {
                rightText = (pt_kr->d_pos.first_alg == true) ? "✔" : "";
                MenuItem::step();
            }
        };

        struct AlgoSelecItem2 : MenuItem {
            K_Rush *pt_kr;
            void onAction(const event::Action &e) override {

                    pt_kr->d_pos.first_alg = false;
            }
            void step() override {
                rightText = (pt_kr->d_pos.first_alg == false) ? "✔" : "";
                MenuItem::step();
            }
        };

        void appendContextMenu(Menu *menu) override{
            K_Rush *pt_kr = dynamic_cast<K_Rush*>(module);
            //assert(pt_kr);
            if( pt_kr){
                menu->addChild(construct<MenuEntry>());
                //menu->addChild(construct<MenuLabel>(&MenuLabel::text, "Not so blank panels"));
                menu->addChild(construct<AlgoSelecItem>(&AlgoSelecItem::text, "1rst Algo",&AlgoSelecItem::pt_kr,pt_kr));
                menu->addChild(construct<AlgoSelecItem2>(&AlgoSelecItem2::text, "2nd Algo",&AlgoSelecItem2::pt_kr,pt_kr));
            }



        }
};









Model *modelK_Rush = createModel<K_Rush, K_RushWidget>("K_Rush");



