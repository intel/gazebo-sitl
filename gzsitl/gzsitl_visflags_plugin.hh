#ifndef GZCOAVTARGET_VPLUGIN_H
#define GZCOAVTARGET_VPLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/rendering/Visual.hh>

namespace gazebo
{
class GZSitlVisibilityFlagsPlugin : public VisualPlugin
{
  public:
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf);

  private:
    rendering::VisualPtr visual;
};
}

#endif

