#include "Edge.hpp"


Plugin *plugin;

void init(rack::Plugin *p) {
	plugin = p;
	p->slug = TOSTRING(SLUG);
	p->version = TOSTRING(VERSION);
	p->addModel (modelWCO_Osc);
	p->addModel (modelBad_Haas);
	p->addModel (modelK_Rush);
}
