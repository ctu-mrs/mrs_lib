from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution, IfElseSubstitution, PythonExpression, TextSubstitution

def sanitize_custom_config_path(custom_config_launch_config: LaunchConfiguration):
    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
        condition=PythonExpression(['"', custom_config_launch_config, '" != "" and ', 'not "', custom_config_launch_config, '".startswith("/")']),
        if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config_launch_config]),
        else_value=custom_config_launch_config
    )
    
    return custom_config