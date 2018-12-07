#!/usr/bin/env python

import sys

from google.protobuf.compiler import plugin_pb2 as plugin
from google.protobuf import json_format as json_f

import jinja2

import itertools
import json
import types
import os
import sys

from google.protobuf.descriptor_pb2 import DescriptorProto, EnumDescriptorProto, ServiceDescriptorProto, FieldDescriptorProto, OneofDescriptorProto

#Class that holds a protobuf message and some other details needed by the generator(jinja2 template).
class DetailedMessage:
    def __init__(self, message=None):
        self.message = message
        self.HasOneOf = "false"
        self.oneOfList = []

#Class that holds a protobuf service and some other details needed by the generator(jinja2 template).
class DetailedPackage:
    def __init__(self, service=None):
        self.name = "NoName"
        self.service = service
        
#JINJA2 function to render a file from a template.
def render(tpl_path, context):
    path, filename = os.path.split(tpl_path)
    return jinja2.Environment(loader=jinja2.FileSystemLoader(path or './')).get_template(filename).render(**context)

#Main plugin function
def generate_code(request, response):
    
    #The context is the object sent to the JINJA2 template
    context = types.SimpleNamespace()
    context.serviceVersion = 1

    context.detailedPackages = []

    MainFilePath = os.path.join(".", "src/main.cpp")
    function_list = []
    fileIndex = 0
    
    for proto_file in request.proto_file:
        context.detailedPackages.append(DetailedPackage())
        context.detailedPackages[fileIndex].name = proto_file.package.split(".")[-1]
        context.detailedPackages[fileIndex].filename = proto_file.name.split(".")[0]
        context.detailedPackages[fileIndex].namespace = proto_file.package.replace(".", "::")
        context.detailedPackages[fileIndex].HasRPC = 0
        context.detailedPackages[fileIndex].HasMessage = 0

        HeaderFilePath = os.path.join(".", "src/node.h")
        CppFilePath = os.path.join(".", "src/node.cpp")

        package_name = str(proto_file.package)

        service_name = proto_file.package.split('.')[-1] + '/'

        #We lower the case to respect ROS coding standard style
        CppProtoConverterFilePath = os.path.join(".", "src/{}_proto_converter.cpp".format(proto_file.name.split(".")[0].lower()))
        HeaderProtoConverterFilePath = os.path.join(".", "src/{}_proto_converter.h".format(proto_file.name.split(".")[0].lower()))
        CppRosConverterFilePath = os.path.join(".", "src/{}_ros_converter.cpp".format(proto_file.name.split(".")[0].lower()))
        HeaderRosConverterFilePath = os.path.join(".", "src/{}_ros_converter.h".format(proto_file.name.split(".")[0].lower()))

        list_detailedMessage = []
        list_detailedMethod = []

        # For every item in the current proto file
        for item, package in traverse(proto_file):
            context.HasOneOf = 0
            
            if isinstance(item, EnumDescriptorProto):
                context.item = item

                ros_enumPath = os.path.join(".", "msg/{}.msg".format(item.name))
                
                with open(ros_enumPath, 'wt') as serviceFile:
                    serviceFile.write(render("./templates/ros_enum.jinja2", context.__dict__))
            #If this it a message
            if isinstance(item, DescriptorProto):
                tempMessage = DetailedMessage(item)
                context.detailedPackages[fileIndex].HasMessage = 1

                for member in item.field:
                    #If a member is part of a oneof, it will have this additional field
                    if member.HasField("oneof_index"):
                        context.HasOneOf = 1
                        tempMessage.HasOneOf = "true"
                    else:
                        context.HasOneOf = 0
                        tempMessage.HasOneOf = "false"
                
                context.item = item

                #If the proto file contains a ONEOF we need to generate a separate file to handle it.
                if context.HasOneOf == 1:

                    #This line gets the list of ONEOF that is in the current message.
                    oneOfList = item.ListFields()[-1][1]
                    
                    tempMessage.oneOfList = item.ListFields()[-1][1]
                    ros_oneofPath = os.path.join(".", "msg/{}_{}.msg".format(item.name, oneOfList[0].name))
                    
                    with open(ros_oneofPath, 'wt') as serviceFile:
                        serviceFile.write(render("./templates/ros_oneof.jinja2", context.__dict__))

                
                list_detailedMessage.append(tempMessage)
                ros_messagePath = os.path.join(".", "msg/{}.msg".format(item.name))
                
                #We call jinja2 to generate a ROS message.
                with open(ros_messagePath, 'wt') as serviceFile:
                    serviceFile.write(render("./templates/ros_message.jinja2", context.__dict__))
                
            #If this is a service (A group of method)
            if isinstance(item, ServiceDescriptorProto):
                for idx, method in enumerate(item.method):
                    context.item = method
                    if "Topic" not in method.name:
                        function_list.append(method.name)
                    ros_servicePath = os.path.join(".", "srv/{}.srv".format(method.name))
                    with open(ros_servicePath, 'wt') as serviceFile:  
                        serviceFile.write(render("./templates/ros_service.jinja2", context.__dict__))
                
                context.detailedPackages[fileIndex].service = item
                context.detailedPackages[fileIndex].HasRPC = 1

        context.currentPackageName = context.detailedPackages[fileIndex].name
        context.currentNamespace = proto_file.package.replace(".", "::")
        context.currentFilename = context.detailedPackages[fileIndex].filename
        context.item = list_detailedMessage

        if context.detailedPackages[fileIndex].HasMessage == 1:
            #Wecall jinja2 to generate a prot/ROS converter for every protobuf message.
            with open(CppProtoConverterFilePath, 'wt') as converterFile:
                converterFile.write(render("./templates/proto_converter.cpp.jinja2", context.__dict__))
            with open(HeaderProtoConverterFilePath, 'wt') as converterFile:
                converterFile.write(render("./templates/proto_converter.h.jinja2", context.__dict__))
            with open(CppRosConverterFilePath, 'wt') as converterFile:
                converterFile.write(render("./templates/ros_converter.cpp.jinja2", context.__dict__))
            with open(HeaderRosConverterFilePath, 'wt') as converterFile:
                converterFile.write(render("./templates/ros_converter.h.jinja2", context.__dict__))

        fileIndex = fileIndex + 1

    context.list_function = function_list

    #We jinja2 to generate the ROS node.
    with open(HeaderFilePath, 'wt') as nodeFile:  
        nodeFile.write(render("./templates/NodeServices.h.jinja2", context.__dict__))    
    with open(CppFilePath, 'wt') as nodeFile:  
        nodeFile.write(render("./templates/NodeServices.cpp.jinja2", context.__dict__))
    with open(MainFilePath, 'wt') as mainFile:
        mainFile.write(render("./templates/main.jinja2", context.__dict__))

def traverse(proto_file):
    #recursive function that browse a protobof item
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

    #return a list of everything found in the proto file
    return itertools.chain(
      _traverse(proto_file.package, proto_file.enum_type),
      _traverse(proto_file.package, proto_file.message_type),
      _traverse(proto_file.package, proto_file.service),
  )

if __name__ == '__main__':
    # Read request message from stdin
    data = sys.stdin.buffer.read()

    # Parse request
    request = plugin.CodeGeneratorRequest()
    request.ParseFromString(data)

    # Create response
    response = plugin.CodeGeneratorResponse()

    # Generate code
    generate_code(request, response)

    # Serialise response message
    output = response.SerializeToString()

    # Write to stdout
    sys.stdout.buffer.write(output)