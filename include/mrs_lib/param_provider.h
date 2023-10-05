#include <string>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <ros/node_handle.h>

namespace mrs_lib
{
  class ParamProvider
  {
    public:

      template <typename T>
      bool getParam(const std::string& param_name, T& value_out)
      {
        for (const auto& json : m_jsons)
        {
          Json::Value cur_node = json[param_name];

          /* // If that fails, try to load it sequentially as a map. */
          /* // (this is for the second case) */
          /* if (!loaded) */
          /* { */
          /*   loaded = true; */
          /*   cur_node.reset(yaml); */
          /*   constexpr char delimiter = '/'; */
          /*   auto substr_start = std::begin(param_name); */
          /*   auto substr_end = substr_start; */
          /*   do */
          /*   { */
          /*     substr_end = std::find(substr_start, std::end(param_name), delimiter);; */
          /*     const auto param_substr = param_name.substr(substr_start - std::begin(param_name), substr_end - std::begin(param_name)); */
          /*     substr_start = substr_end+1; */

          /*     try */
          /*     { */
          /*       cur_node = cur_node[param_substr]; */
          /*     } */
          /*     catch (const YAML::BadSubscript& e) */
          /*     { */
          /*       break; */
          /*       loaded = false; */
          /*     } */
          /*   } */
          /*   while (substr_end != std::end(param_name)); */
          /* } */

          /* if (loaded) */
          /* { */
          /*   try */
          /*   { */
          /*     // similar problem as with checking for a value */
          /*     // try catch is the only option... */
          /*     value_out = cur_node.as<T>(); */
          /*     return true; */
          /*   } */
          /*   catch (const YAML::BadConversion& e) */
          /*   {} */
          /* } */

          value_out = cur_node.as<T>();
          return true;
        }

        if (m_use_rosparam)
          return m_nh.getParam(param_name, value_out);

        return false;
      }

      ParamProvider(const ros::NodeHandle& nh, const bool use_rosparam = true)
        : m_nh(nh), m_use_rosparam(use_rosparam)
      {
      }

      void addYamlFile(const std::string& filepath)
      {
        std::ifstream ifstr(filepath);
        m_jsons.emplace_back();
        ifstr >> m_jsons.back();
      }

    private:

      std::vector<Json::Value> m_jsons;
      ros::NodeHandle m_nh;
      bool m_use_rosparam;
  };
}
