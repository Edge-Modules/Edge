#include "Edge.hpp"


Plugin *plugin;

void init(rack::Plugin *p) {
	plugin = p;
	p->slug = TOSTRING(SLUG);
	p->version = TOSTRING(VERSION);
	p->addModel (modelWTFDoveVCO);
	//p->addModel (modelWTFDoveVCO_IIR);
	p->addModel (modelMyasmaDist);
}
