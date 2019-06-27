#include "Edge.hpp"


Plugin *pluginInstance;

void init(rack::Plugin *p) {
	pluginInstance = p;
	p->addModel (modelWCO_Osc);
	p->addModel (modelK_Rush);
	p->addModel (modelBad_Haas);
	//p->addModel (modelConvolver);
}
