/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#include "gzsitl_visflags_plugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(GZSitlVisibilityFlagsPlugin)

void GZSitlVisibilityFlagsPlugin::Load(rendering::VisualPtr _parent,
                                       sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->visual = _parent;

    // Store pointer to the inner parameters list
    sdf::ElementPtr plugin_params = _sdf;

    // This block is needed because of an inconsistency with the _sdf argument
    // on different calls to Load on Gazebo VisualPlugins
    if (_sdf->HasElement("sdf")) {
        plugin_params = _sdf->GetElement("sdf");
    }

    bool params_found = false;

    // Find visibility flags
    bool vis_all = false;
    if (plugin_params->HasElement("gz_visibility_all")) {
        vis_all = plugin_params->Get<bool>("gz_visibility_all");
        params_found = true;
    }

    bool vis_gui = false;
    if (plugin_params->HasElement("gz_visibility_gui")) {
        vis_gui = plugin_params->Get<bool>("gz_visibility_gui");
        params_found = true;
    }

    bool vis_sel = false;
    if (plugin_params->HasElement("gz_visibility_sel")) {
        vis_sel = plugin_params->Get<bool>("gz_visibility_sel");
        params_found = true;
    }

    // Don't change anything if flags have not been found
    if (!params_found) {
        return;
    }

    int visflags = (vis_all * GZ_VISIBILITY_ALL) |
                   (vis_sel * GZ_VISIBILITY_SELECTABLE) |
                   (vis_gui * GZ_VISIBILITY_GUI);

    // Set visibility flags
    this->visual->SetVisibilityFlags(visflags);
}

