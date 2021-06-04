#!/usr/bin/env python3
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed 
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys

from google.protobuf.compiler import plugin_pb2 as plugin
from google.protobuf import json_format as json_f

import jinja2

import itertools
import json
import types
import os
import shutil
import sys
import re
from pathlib import PurePath

from enum import Enum
from collections import OrderedDict

from google.protobuf.descriptor_pb2 import DescriptorProto, EnumDescriptorProto, ServiceDescriptorProto, FieldDescriptorProto, OneofDescriptorProto
from google.protobuf.descriptor import FieldDescriptor
# Some names cause generation bugs and we have to hardcode them in here 
# Reboot in Base.proto auto-generates the RebootRequest and RebootResponse classes, but RebootResponse RPC already exists in DeviceConfig.proto
# Refresh* RPCs have to be called at exactly 1kHz and ROS 1 cannot achieve this so they are removed
# RPCs for internal use have also been removed from the ROS driver
FORBIDDEN_RPC_METHODS = ["Reboot", \
                        "Refresh", "RefreshCommand", "RefreshCustomData", "RefreshFeedback", \
                        "ReadTorqueCalibration", "WriteTorqueCalibration", "GetVectorDriveParameters", "SetVectorDriveParameters", "GetEncoderDerivativeParameters", "SetEncoderDerivativeParameters", "StartFrequencyResponse", "StopFrequencyResponse", "StartStepResponse", "StopStepResponse", "StartRampResponse", "StopRampResponse", \
                        "ReadCapSenseRegister", "WriteCapSenseRegister"]

# There are some packages we don't want to generate because the use of their RPC's as ROS services doesn't make sense at all and could cause some undefined behaviour
NON_GENERATED_PACKAGES = ["Session"]
PROTO_FILES_RELATIVE_PATH = PurePath("..", "protos")

class DetailedOneOf:
    '''
    Class that holds a one_of and the fields in the one_of

    Attributes : 
    - name:  Name of the one_of [string]
    - fields: Protobuf FieldDescriptor's that are in the one_of [list of FieldDescriptorProto]
    - duplicated_fields: Names of the fields that are duplicated in the generation context (ex. Feedback) [list of strings] 

    Usage : Create the DetailedOneOf with its name, then use the addField() method to populate the fields list (automatically populates the duplicated_fields list)
    '''
    def __init__(self, one_of_name = ""):
        '''
        Constructor for the DetailedOneOf

        Arguments:
        - one_of_name: Name of the one_of [string]
        '''
        self.name = one_of_name
        self.fields = []
        self.duplicated_fields = []

    def addField(self, field, is_field_duplicated):
        '''
        Adds a field to the one_of
        
        Arguments: 
        - field: Field to add [FieldDescriptor]
        - is_field_duplicated: True if the field_type is duplicated in the generation context (ex. Feedback) [bool]
        
        Returns: None
        '''
        self.fields.append(field)
        if is_field_duplicated:
            self.duplicated_fields.append(field.type_name.split(".")[-1])

class DetailedMessage:
    '''
    Class that holds a message or an enum (all but a ServiceDescriptorProto)

    Attributes: 
    - name:  Name of the message or enum [string]
    - name_lowercase_with_underscores: Name of the message in lowercase with underscores (following the ROS convention) [string]
    - message: Protobuf Descriptor [DescriptorProto]
    - cpp_namespace: Full C++ namespace for the message (ex. Kinova::Api::Common) [string]
    - full_proto_package_name: Full name of the Protobuf package with point delimiters (ex. Kinova.Api.Common) [string]
    - duplicated_fields: Names of the fields that are duplicated in the generation context (ex. Feedback) [list of strings] 
    - containing_folder: Name of the subfolder in which the msg file will be generated, determined with the C++ namespace (ex. "ActuatorCyclic/"). If the namespace is Kinova::Api, then subfolder is equal to an empty string. [string]
    - prepend_message_name: If the message is duplicated in the generation context, this attribute is in the form of "ActuatorCyclic_". This is prepended to the message name to ensure there is no naming clash during the ROS message generation. [string]
    - one_of_list: List of DetailedOneOf's present in the message fields. [list of DetailedOneOf]

    Usage: Create the DetailedMessage, then use the addDetailedOneOf() method to populate the one_of_list attribute
    '''
    def __init__(self, message, package, is_message_duplicated, duplicatedFields=[]):
        '''
        Constructor for the DetailedMessage

        Arguments:
        - message: Protobuf object for the message [DescriptorProto]
        - package: Full name of the message's package (ex. "Kinova.Api.Common") [string]
        - is_message_duplicated: True if the message type is duplicated in the generation context (ex. Feedback) [bool]
        - duplicated_fields: Names of the fields that are duplicated in the generation context (ex. Feedback) (empty by default) [list of strings] 
        '''
        self.name = message.name
        self.name_lowercase_with_underscores = '_'.join(re.findall('[A-Z][^A-Z]*', self.name)).lower()
        self.message = message

        self.cpp_namespace = package.replace(".", "::")
        self.full_proto_package_name = package

        # Some fields's type may be duplicated messages
        self.duplicated_fields = []
        for df in duplicatedFields:
            self.duplicated_fields.append(df.type_name.split(".")[-1])

        # Sub-folder in which messages are generated
        # Corresponds to the Protobuf Service they relate to
        # Those without a service (Frame, Errors) are generated directly in /msg
        self.containing_folder = "_".join(re.findall('[A-Z][^A-Z]*', package.split(".")[-1])).lower() + "/"
        if self.containing_folder == "api/":
            self.containing_folder = ""

        # If it is a duplicated message, we prepend it with the Protobuf Service name (because ROS messages don't handle C++ namespaces)
        if is_message_duplicated:
            self.prepend_message_name = package.split(".")[-1] + "_"
        else:
            self.prepend_message_name = ""
        
        # List of all the DetailedOneof's in the message
        self.one_of_list = []

    # Method to add the DetailedOneOf's in the DetailedMessage
    def addDetailedOneOf(self, one_of):
        '''
        Adds a DetailedOneOf to the message's one_of_list
        
        Arguments: 
        - one_of: One_of to add to the message [DetailedOneOf]
        
        Returns: None
        '''
        self.one_of_list.append(one_of)

    def isEnum(self):
        '''
        Returns True if the message is an enum (Protobuf message is of type EnumDescriptorProto)

        Arguments: None

        Returns: Boolean
        '''
        return isinstance(self.message, EnumDescriptorProto)

# Class that holds a protobuf service and some other details needed by the generator(Jinja2 template)
class DetailedPackage:
    '''
    Class that holds a Protobuf package

    Attributes: 
    - name: Name of the package delimited with points (ex. "Kinova.Api.Common") [string]
    - short_name: Short name of the package (ex. "Common" for "Kinova.Api.Common") [string]
    - short_name_lowercase_with_underscores: Short name of the package in lowercase with underscores (following the ROS convention) [string]
    - cpp_namespace: Full C++ namespace for the package (ex. Kinova::Api::Common) [string]
    - messages: List of messages in the package [list of DetailedMessage]
    - methods: List of RPC methods in the package [list of DetailedRPC]
    - enums: List of enums in the package [list of DetailedMessage]

    Usage: Create the DetailedPackage, then add the messages, enums and RPC's with the corresponding methods.
    '''
    def __init__(self, package):
        '''
        Constructor for the DetailedPackage

        Arguments:
        - package: Full name of the message's package (ex. "Kinova.Api.Common") [string]
        '''
        self.name = package
        self.short_name = self.name.split(".")[-1]
        self.short_name_lowercase_with_underscores = '_'.join(re.findall('[A-Z][^A-Z]*', self.short_name)).lower()
        self.cpp_namespace = self.name.replace(".", "::")
        self.messages = [] # list of DetailedMessage
        self.methods = []  # list of DetailedRPC
        self.enums = []    # list of DetailedMessage
    
    def addMessage(self, detailed_message):
        '''
        Adds a message to the package's messages list
        
        Arguments: 
        - detailed_message: Message to add to the package [DetailedMessage]
        
        Returns: None
        '''
        self.messages.append(detailed_message)

    def addRPC(self, detailed_rpc):
        '''
        Adds a RPC to the package's methods list
        
        Arguments: 
        - detailed_rpc: RPC to add to the package [DetailedRPC]
        
        Returns: None
        '''
        self.methods.append(detailed_rpc)

    def addEnum(self, detailed_enum):
        '''
        Adds an enum to the package's enums list
        
        Arguments: 
        - detailed_enum: Enum to add to the package [DetailedMessage]
        
        Returns: None
        '''
        self.enums.append(detailed_enum)

class DetailedRPC:
    '''
    Class that holds a Protobuf RPC

    Attributes: 
    - name: Name of the RPC [string]
    - name_lowercase_with_underscores: Name of the RPC in lowercase with underscores (following the ROS convention) [string]
    - rpc: Protobuf object for the RPC [ServiceDescriptorProto]
    - cpp_namespace: Full C++ namespace for the RPC (ex. Kinova::Api::BaseCyclic) [string]
    - full_proto_package_name: Full name of the Protobuf package with point delimiters (ex. Kinova.Api.Common) [string]
    - is_rpc_duplicated: True if the message type is duplicated in the generation context (ex. Feedback) [bool]
    - prepend_rpc_package_name: If the RPC is duplicated in the generation context, this string will be populated with the RPC package's short name (ex. "BaseCyclic_" for Feedback RPC). Empty otherwise. [string]
    
    - is_input_type_duplicated: True if the input type of the RPC is duplicated in the generation context [bool]
    - ros_service_input_name: Type of the ROS service input, modified according to the is_input_type_duplicated parameter [string]
    - input_type_short_name: Short name of the input (ex. "Command" for "Kinova.Api.BaseCyclic.Command") [string]
    - input_type_cpp_namespace: Full C++ namespace for the RPC (ex. Kinova::Api::BaseCyclic) [string]
    
    - is_output_type_duplicated: True if the output type of the RPC is duplicated in the generation context [bool]
    - ros_service_output_name: Type of the ROS service output, modified according to the is_output_type_duplicated parameter [string]
    - output_type_short_name: Short name of the output (ex. "Feedback" for "Kinova.Api.BaseCyclic.Feedback") [string]
    - output_type_cpp_namespace: Full C++ namespace for the RPC (ex. Kinova::Api::BaseCyclic) [string]

    - is_notification_rpc: True if the RPC is a Notification RPC (ends with "Topic")
    - prepend_on_notification: If the RPC is a Notification RPC, is put to "OnNotification". Otherwise, it is empty [string]
    - notification_message_cpp_namespace: Full C++ namespace of the message that corresponds to the Notification RPC ("SafetyNotification" for the RPC "SafetyTopic") [string]
    '''
    def __init__(self, rpc, package, is_rpc_duplicated, is_rpc_deprecated, is_input_type_duplicated, is_output_type_duplicated, notification_message_cpp_namespace=None):
        '''
        Constructor for the DetailedRPC

        Arguments:
        - rpc: Protobuf object for the RPC [ServiceDescriptorProto]
        - package: Full name of the message's package (ex. "Kinova.Api.Common") [string]
        - is_rpc_duplicated: True if the message type is duplicated in the generation context (ex. Feedback) [bool]
        - is_rpc_deprecated: True if the message type is deprecated in the C++ Kortex API [bool]
        - is_input_type_duplicated: True if the input type of the RPC is duplicated in the generation context [bool]
        - is_output_type_duplicated: True if the output type of the RPC is duplicated in the generation context [bool]
        '''
        self.name = rpc.name
        self.name_lowercase_with_underscores = '_'.join(re.findall('[A-Z][^A-Z]*', self.name)).lower()
        self.rpc = rpc
        self.is_rpc_deprecated = is_rpc_deprecated

        self.cpp_namespace = package.replace(".", "::")
        self.full_proto_package_name = package

        # If it is a duplicated RPC, we prepend it with the Protobuf Service name (because ROS services don't handle C++ namespaces)
        if is_rpc_duplicated:
            self.prepend_rpc_package_name = package.split(".")[-1] + "_"
        else:
            self.prepend_rpc_package_name = ""

        # Helper variables for input and output types
        if is_input_type_duplicated:
            self.ros_service_input_name  = rpc.input_type.split(".")[-2] + "_" + rpc.input_type.split(".")[-1]
        else:
            self.ros_service_input_name  = rpc.input_type.split(".")[-1]


        self.input_type_short_name = rpc.input_type.split(".")[-1]
        self.input_type_cpp_namespace = "::".join(rpc.input_type.split(".")[:-1])[2:]

        if is_output_type_duplicated:
            self.ros_service_output_name  = rpc.output_type.split(".")[-2] + "_" + rpc.output_type.split(".")[-1]
        else:
            self.ros_service_output_name  = rpc.output_type.split(".")[-1]

        self.output_type_short_name = rpc.output_type.split(".")[-1]
        self.output_type_cpp_namespace = "::".join(rpc.output_type.split(".")[:-1])[2:]

        # Notifications have the word 'Topic' at the end of the RPC name
        self.is_notification_rpc = re.match(r"\w+Topic", rpc.name)
        self.notification_message_cpp_namespace = ""
        self.prepend_on_notification = "OnNotification" if self.is_notification_rpc else ""

    def set_notification_cpp_namespace(self, notification_message_cpp_namespace):
        '''
        Setter for the DetailedRPC's notification namespace
        
        Arguments:
        - notification_message_cpp_namespace: C++ namespace for the Notification message that goes with the Notification RPC (optional) [string]
        '''
        self.notification_message_cpp_namespace = notification_message_cpp_namespace

# Jinja2 function to render a file from a template
def render(tpl_path, context):
    path, filename = os.path.split(tpl_path)
    return jinja2.Environment(loader=jinja2.FileSystemLoader(path or './')).get_template(filename).render(**context)

# Main plugin function
def generate_code(request, response):
    
    # MainFilePath = os.path.join(".", "src/main.cpp")
    # Find all proto files
    file_map = OrderedDict()
    for proto_file in request.proto_file:
        file_map[proto_file.name] = proto_file

    # Create an ordered dictionary for all Protobuf packages 
    packages_dict = OrderedDict()
    
    # Find Messages and RPC's that have the same name within the proto files because ROS doesn't handle namespaces
    messages_unordered_set = set()
    rpcs_unordered_set = set()
    duplicated_messages_unordered_set = set()
    duplicated_rpcs_unordered_set = set()
    deprecated_rpcs_dict_of_lists = OrderedDict()

    # Find the *Notification messages to match them with their On*Topic RPCs
    # key example: "Kinova.Api.Package.MyGivenTopic"
    # value example: "Kinova.Api.Package.MyGivenNotification"
    notification_messages_map = {}

    for filename, proto_file in file_map.items():
        # Traverse proto files with the recursive calls
        for item, package in traverse(proto_file):
            
            # Skip the packages we don't want to generate
            if package.split(".")[-1] in NON_GENERATED_PACKAGES:
                continue

            packages_dict[package] = DetailedPackage(package)
            # If the item is a message or an enum
            if not isinstance(item, ServiceDescriptorProto):
                if item.name.casefold() not in messages_unordered_set:
                    messages_unordered_set.add(item.name.casefold())
                else:
                    duplicated_messages_unordered_set.add(item.name.casefold())
            # If the item is a Protobuf service (a collection of methods)
            else: 
                for method in item.method:
                    if method.name not in rpcs_unordered_set:
                        rpcs_unordered_set.add(method.name)
                    else:
                        duplicated_rpcs_unordered_set.add(method.name)

        # Parse through the proto files to find deprecation tags that the recursive function cannot find
        proto_file_reader = open(str(PurePath(PROTO_FILES_RELATIVE_PATH, filename)), encoding='utf8', mode='r')
        filename_without_extension = filename.replace('.proto', '') 
        deprecated_rpcs_dict_of_lists[filename_without_extension] = list()
        RPC_REGULAR_EXPRESSION = r'.*rpc (\w*) * \((\S*)\) *returns* \((\S*)\).*'
        text_line = "_"
        while text_line != "":
            text_line = proto_file_reader.readline()
            if "rpc " in text_line:
                # Find deprecation tags
                if re.search(r'@DEPRECATED', text_line):
                    rpc_name = re.sub(RPC_REGULAR_EXPRESSION, r'\1', text_line).strip()
                    deprecated_rpcs_dict_of_lists[filename_without_extension].append(rpc_name)
                # Find notification tags
                m = re.search(r"(\w+Topic).*@PUB_SUB=(.*Notification)", text_line)
                if m:
                    # Name is already complete
                    if "." in m.group(2):
                        notif = m.group(2)
                    # Name is relative to current package
                    else:
                        notif = "{}.{}".format(package, m.group(2))
                    key = "{}.{}".format(package, m.group(1))
                    notification_messages_map[key] = notif

        # print (deprecated_rpcs_dict_of_lists.items())

    # Remove old generated files and create new directories
    for package in packages_dict.values():
        for s in ['srv', 'msg']:
            shutil.rmtree("../{}/generated/{}".format(s, package.short_name_lowercase_with_underscores), ignore_errors=True)
            os.makedirs("../{}/generated/{}".format(s, package.short_name_lowercase_with_underscores))
    shutil.rmtree("../src/generated", ignore_errors=True)
    shutil.rmtree("../include/kortex_driver/generated", ignore_errors=True)
    os.makedirs("../src/generated/robot")
    os.makedirs("../src/generated/simulation")
    os.makedirs("../include/kortex_driver/generated/interfaces")
    os.makedirs("../include/kortex_driver/generated/robot")
    os.makedirs("../include/kortex_driver/generated/simulation")

    ###########################################
    # Parse the proto files to add the messages and RPC's to the DetailedPackage's
    for filename, proto_file in file_map.items():

        # For every item in the current proto file
        for item, package in traverse(proto_file):

            # Skip the packages we don't want to generate
            if package.split(".")[-1] in NON_GENERATED_PACKAGES:
                continue
            
            current_package = packages_dict[package]

            # If this is an enum
            if isinstance(item, EnumDescriptorProto):
                is_enum_duplicated = item.name.casefold() in duplicated_messages_unordered_set
                current_package.addEnum(DetailedMessage(item, package, is_enum_duplicated))
                
            # If this is a message
            if isinstance(item, DescriptorProto):
                is_message_duplicated = item.name.casefold() in duplicated_messages_unordered_set
                duplicated_fields = filter(lambda x : x.type_name.split(".")[-1].casefold() in duplicated_messages_unordered_set, item.field)
                temp_message = DetailedMessage(item, package, is_message_duplicated, duplicated_fields)
                # Find if the message contains oneof
                message_contains_one_of = False
                for member in item.field:
                    # If a member is part of a one_of, it will have this additional field.
                    if member.HasField("oneof_index"):
                        message_contains_one_of = True
                        break

                # Register every one_of in the message
                if message_contains_one_of:

                    # Find the one_of names
                    for one_of in item.ListFields()[-1][1]: # This is the very obscure way to get the one_of's name
                        temp_message.one_of_list.append(DetailedOneOf(one_of.name))
    
                    # Find the fields and what one_of they belong to
                    for member in item.field:
                        # If a member is part of a one_of, add it to the DetailedOneOf it belongs to
                        if member.HasField("oneof_index"):
                            is_field_duplicated = member.type_name.split(".")[-1].casefold() in duplicated_messages_unordered_set
                            temp_message.one_of_list[member.oneof_index].addField(member, is_field_duplicated)

                current_package.addMessage(temp_message)

            # If this is a Protobuf service (a group of RPC's)
            if isinstance(item, ServiceDescriptorProto):
                # Get sublist of all deprecated methods in this service
                deprecated_rpcs_in_this_service = deprecated_rpcs_dict_of_lists[item.name]
                # Register every RPC in the Protobuf service
                for rpc in item.method:
                    # Do not generate the services that cause generation bugs
                    if rpc.name in FORBIDDEN_RPC_METHODS:
                        continue
                    is_rpc_duplicated = rpc.name in duplicated_rpcs_unordered_set
                    is_input_type_duplicated = rpc.input_type.split(".")[-1].casefold() in duplicated_messages_unordered_set
                    is_output_type_duplicated = rpc.output_type.split(".")[-1].casefold() in duplicated_messages_unordered_set
                    is_rpc_deprecated = True if rpc.name in deprecated_rpcs_in_this_service else False
                    temp_rpc = DetailedRPC(rpc, package, is_rpc_duplicated, is_rpc_deprecated, is_input_type_duplicated, is_output_type_duplicated)
                    
                    # Add Notification C++ namespace to RPC if it is a notification RPC
                    if temp_rpc.is_notification_rpc:
                        notification_message = notification_messages_map["{}.{}".format(package, rpc.name)]
                        notification_message_cpp_namespace = "::".join(notification_message.split('.')[:-1])
                        temp_rpc.set_notification_cpp_namespace(notification_message_cpp_namespace)

                    current_package.addRPC(temp_rpc)

    ###########################################
    # Generate the include names with the packages that contain messages
    packages_with_messages = filter(lambda x: len(x.messages) > 0 , packages_dict.values())
    include_file_names = []
    for p in packages_with_messages:
        for s in ["proto", "ros"]:
            include_file_names.append("kortex_driver/generated/robot/{}_{}_converter.h".format(p.short_name.lower(), s))

    # Generate the ROS files for each package
    for package in packages_dict.values():

        # Generate the enums
        for enum in package.enums:
            this_enum_context = types.SimpleNamespace()
            this_enum_context.item = enum
            ros_enum_path = os.path.join("..", "msg/generated/{}{}{}.msg".format(enum.containing_folder, enum.prepend_message_name, enum.name))
            with open(ros_enum_path, 'wt') as serviceFile:
                serviceFile.write(render("../templates/ros_enum.msg.jinja2", this_enum_context.__dict__))
        
        # Generate the messages
        for message in package.messages:
            this_message_context = types.SimpleNamespace()
            this_message_context.item = message
            this_message_context.field_descriptor_class = FieldDescriptor
            
            # Generate the one_of's for the message
            for detailed_one_of in message.one_of_list: # not empty
                this_message_context.detailed_one_of = detailed_one_of
                ros_oneofPath = os.path.join("..", "msg/generated/{}{}{}_{}.msg".format(message.containing_folder, message.prepend_message_name, message.name, detailed_one_of.name))
                with open(ros_oneofPath, 'wt') as serviceFile:
                    serviceFile.write(render("../templates/ros_oneof.msg.jinja2", this_message_context.__dict__))

            # Generate the message
            ros_messagePath = os.path.join("..", "msg/generated/{}{}{}.msg".format(message.containing_folder, message.prepend_message_name, message.name))                
            with open(ros_messagePath, 'wt') as serviceFile:
                serviceFile.write(render("../templates/ros_message.msg.jinja2", this_message_context.__dict__))

        # Generate the RPC's
        for rpc in package.methods:
            this_rpc_context = types.SimpleNamespace()
            this_rpc_context.item = rpc
            ros_servicePath = os.path.join("..", "srv/generated/{}/{}{}{}.srv".format(package.short_name_lowercase_with_underscores, rpc.prepend_rpc_package_name, rpc.prepend_on_notification, rpc.name))
            with open(ros_servicePath, 'wt') as serviceFile:  
                serviceFile.write(render("../templates/ros_service.srv.jinja2", this_rpc_context.__dict__))

        # Generate the Proto-ROS converters (C++ files)
        this_package_context = types.SimpleNamespace()
        this_package_context.package = package
        
        if package.messages: # package contains at least one message
            # Proto converter header file
            current_header_filename = "kortex_driver/generated/robot/{}_proto_converter.h".format(package.short_name.lower())
            this_package_context.include_file_names = filter(lambda x : "proto_converter" in x and x != current_header_filename, include_file_names)
            with open(os.path.join("..", "include/" + current_header_filename), 'wt') as converterFile:
                converterFile.write(render("../templates/proto_converter.h.jinja2", this_package_context.__dict__))
            # Proto converter cpp file
            this_package_context.current_header_filename = current_header_filename
            with open(os.path.join("..", "src/generated/robot/{}_proto_converter.cpp".format(package.short_name.lower())), 'wt') as converterFile:
                converterFile.write(render("../templates/proto_converter.cpp.jinja2", this_package_context.__dict__))
            # ROS converter header file
            current_header_filename = "kortex_driver/generated/robot/{}_ros_converter.h".format(package.short_name.lower())
            this_package_context.include_file_names = filter(lambda x : "ros_converter" in x and x != current_header_filename, include_file_names)
            with open(os.path.join("..", "include/" + current_header_filename), 'wt') as converterFile:
                converterFile.write(render("../templates/ros_converter.h.jinja2", this_package_context.__dict__))
            # ROS converter cpp file
            this_package_context.current_header_filename = current_header_filename
            with open(os.path.join("..", "src/generated/robot/{}_ros_converter.cpp".format(package.short_name.lower())), 'wt') as converterFile:
                converterFile.write(render("../templates/ros_converter.cpp.jinja2", this_package_context.__dict__))

        # Generate the ServiceProxy's for every Kortex API method
        if package.methods: # package contains at least one RPC
            # Generate interface files
            current_header_filename = "kortex_driver/generated/interfaces/{}_services_interface.h".format(package.short_name.lower())
            current_interface_header_filename = current_header_filename
            this_package_context.current_interface_header_filename = current_interface_header_filename
            with open(os.path.join("..", "include/" + current_header_filename), 'wt') as services_file:  
                services_file.write(render("../templates/services_interface.h.jinja2", this_package_context.__dict__))
            
            # Generate robot files
            current_header_filename = "kortex_driver/generated/robot/{}_services.h".format(package.short_name.lower())
            this_package_context.current_header_filename = current_header_filename
            this_package_context.include_file_names = include_file_names
            with open(os.path.join("..", "include/" + current_header_filename), 'wt') as services_file:  
                services_file.write(render("../templates/services_robot.h.jinja2", this_package_context.__dict__))
            with open(os.path.join("..", "src/generated/robot/{}_services.cpp".format(package.short_name.lower())), 'wt') as services_file:  
                services_file.write(render("../templates/services_robot.cpp.jinja2", this_package_context.__dict__))

            # Generate simulation files
            current_header_filename = "kortex_driver/generated/simulation/{}_services.h".format(package.short_name.lower())
            this_package_context.current_header_filename = current_header_filename
            this_package_context.include_file_names = include_file_names
            with open(os.path.join("..", "include/" + current_header_filename), 'wt') as services_file:  
                services_file.write(render("../templates/services_simulation.h.jinja2", this_package_context.__dict__))
            with open(os.path.join("..", "src/generated/simulation/{}_services.cpp".format(package.short_name.lower())), 'wt') as services_file:  
                services_file.write(render("../templates/services_simulation.cpp.jinja2", this_package_context.__dict__))

    # Delete unused folders we created for None
    for package in packages_dict.values():
        for s in ['srv', 'msg']:
            if len(os.listdir('../{}/generated/{}'.format(s, package.short_name_lowercase_with_underscores))) == 0:
                shutil.rmtree("../{}/generated/{}".format(s, package.short_name_lowercase_with_underscores))

def traverse(proto_file):
    # recursive function that browses a protobof item
    def _traverse(package, items):
        for item in items:
            yield item, package
            
            if isinstance(item, DescriptorProto):
                for enum in item.enum_type:
                    yield enum, package

                for nested in item.nested_type:
                    nested_package = package + item.name

                    for nested_item in _traverse(nested, nested_package):
                        yield nested_item, nested_package
            if isinstance(item, ServiceDescriptorProto):
                for rpc in item.method:
                    yield rpc, package

    # returns a list of everything found in the proto file
    return itertools.chain(
      _traverse(proto_file.package, proto_file.enum_type),
      _traverse(proto_file.package, proto_file.message_type),
      _traverse(proto_file.package, proto_file.service),
  )

if __name__ == '__main__':
    # reads request message from stdin
    data = sys.stdin.buffer.read()

    # parses request
    request = plugin.CodeGeneratorRequest()
    request.ParseFromString(data)

    # creates response
    response = plugin.CodeGeneratorResponse()

    # generates code
    generate_code(request, response)

    # serialises response message
    output = response.SerializeToString()

    # writes to stdout
    sys.stdout.buffer.write(output)