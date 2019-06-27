#pragma once
#include "componentlibrary.hpp"
#include <vector>
#include <jansson.h>
//#include "widgets.hpp"
#include <iostream>


namespace rack {

    struct EdgeBlueKnob : RoundKnob {
        EdgeBlueKnob() {
            setSvg(APP->window->loadSvg(asset::plugin(pluginInstance,"res/RoundBlueKnob.svg")));
        }
    };

    struct EdgeRedKnob : RoundKnob {
        EdgeRedKnob() {
            setSvg(APP->window->loadSvg(asset::plugin(pluginInstance,"res/RoundRedKnob.svg")));
        }
    };

}
