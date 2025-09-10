#!/usr/bin/env python3

import yaml
import os
from typing import List, Tuple
from launch_ros.descriptions import ComposableNode
from launch.launch_context import LaunchContext
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.logging import get_logger

class RemappingsCustomConfigParser(LaunchDescriptionEntity):
    """
    Launch component that parses config YAML and adds remappings to a ComposableNode.
    """
    
    def __init__(self, composable_node: ComposableNode, custom_config: LaunchConfiguration, prefix: str = ''):
        """
        Args:
            composable_node: The ComposableNode to wrap
            custom_config: LaunchConfiguration('custom_config') 
            config_namespace: Namespace in config file (e.g., '/uav1/flame_ros')
        """
        self.wrapped_node = composable_node
        self.custom_config = custom_config
        self.prefix = prefix
    
    # Pull out the remappings from the config file. They are list of tuples of two strings.
    def parse_custom_config(self, context: LaunchContext, _custom_config_file: str, prefix: str = ''):
        remappings = []
        if _custom_config_file != '':
            with open(_custom_config_file, 'r') as f:
                yaml_data = yaml.load(f, Loader=yaml.FullLoader)

                # This first part is for the ordinary nodes. Only those have 'ros__parameters'.
                if prefix in yaml_data and 'ros__parameters' in yaml_data[prefix] and 'remappings' in yaml_data[prefix]['ros__parameters']:
                    remappings_subyaml = yaml_data[prefix]['ros__parameters']['remappings']
                    for orig_name in remappings_subyaml:
                        new_name = remappings_subyaml[orig_name]
                        remappings.append((orig_name, new_name))
                else:   # This second part is for the composable nodes. The prefix here is optional, just for separating different node remappings.
                    if prefix == '':    
                        if 'remappings' in yaml_data:
                            remappings_subyaml = yaml_data['remappings']
                            for orig_name in remappings_subyaml:
                                new_name = remappings_subyaml[orig_name]
                                remappings.append((orig_name, new_name))
                        else:
                            get_logger('launch.remappings').warning(msg=f"remappings not found in the config")
                    else:
                        if prefix in yaml_data:
                            if 'remappings' in yaml_data[prefix]:
                                remappings_subyaml = yaml_data[prefix]['remappings']
                                for orig_name in remappings_subyaml:
                                    new_name = remappings_subyaml[orig_name]
                                    remappings.append((orig_name, new_name))
                            else:
                                get_logger('launch.remappings').warning(msg=f"remappings not found in the config under the prefix '{prefix}'")
                        else:
                            get_logger('launch.remappings').warning(msg=f"prefix '{prefix}' not found in the config")
                                                
        return remappings
    
    # ComposableNode needs to have configs in the particular format:
    #   [
    #       ( (TextSubstitution(text='/topic_1'),), (TextSubstitution(text='/remapped_topic_1'),) ),
    #       ( (TextSubstitution(text='/topic_2'),), (TextSubstitution(text='/remapped_topic_2'),) )
    #   ]
    #
    # This method is converting raw list of tuples into this format.
    def convert_remappings(self, raw_remappings: List):
        config_remappings = []
        
        for remap_tuple in raw_remappings:
            config_remappings.append(((TextSubstitution(text=remap_tuple[0]),), (TextSubstitution(text=remap_tuple[1]),)))
            
        return config_remappings
    
    def log_remappings(self, context: LaunchContext, raw_remappings: List, prefix: str = ''):
        if prefix == '':
            get_logger('launch.remappings').info(msg=f"remappings:")
        else:
            get_logger('launch.remappings').info(msg=f"remappings({prefix}):")
            
        for remapping in raw_remappings:
            get_logger('launch.remappings').info(msg=f"\t{remapping[0]} -> {remapping[1]}")
    
    def visit(self, context: LaunchContext):
        """Parse config file and add remappings, then visit wrapped node."""
        
        # Get config file path
        custom_config = self.custom_config.perform(context)
        get_logger('launch.remappings').info(msg=f"custom config path: {custom_config}")
        
        # Take list of tuples from previous step and convert them into list of tuples of tuples of the TextSubstitutions.
        raw_remappings = self.parse_custom_config(context, custom_config, self.prefix)
        self.log_remappings(context, raw_remappings, self.prefix)
        
        # Parse YAML and get remappings
        #config_remappings = [((TextSubstitution(text='/uav1/image_raw'),), (TextSubstitution(text='/some_topic_name'),))]    # remappings data structure
        config_remappings = self.convert_remappings(raw_remappings)
        
        # We have to explicitly test the return value, because if the remappings are not set, the attribute still exist, it is just NoneType.
        old_remappings_attr = getattr(self.wrapped_node, '_ComposableNode__remappings', [])
        old_remappings = list([] if old_remappings_attr is None else old_remappings_attr)
        new_remappings = []
        
        # We have to build the list of remappings 'backwards' - first the new remappings that should override
        # the old ones and then the old ones.
        
        # Add config remappings first
        for t in config_remappings:
            new_remappings.append(t)
            
        # Add existing remappings
        for t in old_remappings:
            new_remappings.append(t)
        
        # Assign to wrapped node
        self.wrapped_node._ComposableNode__remappings = new_remappings