#pragma once
#include "componentlibrary.hpp"
#include <vector>
#include <jansson.h>
#include "widgets.hpp"
#include <iostream>

using namespace std;

namespace rack {

    struct EdgeBlueKnob : RoundKnob {
        EdgeBlueKnob() {
            setSVG(SVG::load(assetPlugin(plugin,"res/RoundBlueKnob.svg")));
        }
    };

    struct EdgeRedKnob : RoundKnob {
        EdgeRedKnob() {
            setSVG(SVG::load(assetPlugin(plugin,"res/RoundRedKnob.svg")));
        }
    };

}
