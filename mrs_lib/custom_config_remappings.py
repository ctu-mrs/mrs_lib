#!/usr/bin/env python3

import yaml
import os
from typing import List, Tuple
from launch_ros.descriptions import ComposableNode
from launch.launch_context import LaunchContext
from launch.substitutions import TextSubstitution
from launch.launch_description_entity import LaunchDescriptionEntity

class RemappingWrapper(LaunchDescriptionEntity):
    """
    Launch component that parses config YAML and adds remappings to a ComposableNode.
    """
    
    #def __init__(self, composable_node: ComposableNode, custom_config, config_namespace: str):
    def __init__(self, composable_node: ComposableNode):
        """
        Args:
            composable_node: The ComposableNode to wrap
            custom_config: LaunchConfiguration('custom_config') 
            config_namespace: Namespace in config file (e.g., '/uav1/flame_ros')
        """
        self._wrapped_node = composable_node
        #self._custom_config = custom_config
        #self._config_namespace = config_namespace
    
    # def __getattr__(self, name):
    #     """Pass through all attributes to the wrapped node."""
    #     return getattr(self._wrapped_node, name)
    
    def visit(self, context: LaunchContext):
        """Parse config file and add remappings, then visit wrapped node."""
        
        # Get config file path
        #config_path = self._custom_config.perform(context)
        
        # Parse YAML and get remappings
        config_remappings = [((TextSubstitution(text='/uav1/image_raw'),), (TextSubstitution(text='/uekjncdks'),))]
        
        # Add to wrapped node using your working pattern
        old_remappings = list(getattr(self._wrapped_node, '_ComposableNode__remappings', []))
        new_remappings = []
        
        # Add config remappings first
        for t in config_remappings:
            new_remappings.append(t)
            
        # Add existing remappings
        for t in old_remappings:
            new_remappings.append(t)
        
        # Assign to wrapped node
        self._wrapped_node._ComposableNode__remappings = new_remappings
        
        # Visit wrapped node
        #return self._wrapped_node.visit(context)