#pragma once
#include "ProtoConverter.h"
#include "template.pb.h"

namespace FabByExample {
	class Template;
	class KinChain;
	class TemplateProtoConverter : public ProtoConverter<proto::TemplateSet>
	{
	public:
		TemplateProtoConverter();
		~TemplateProtoConverter();

		/**
		convert a template to proto, the caller is responsible for freeing the object
		*/
		proto::TemplateSet* ConvertToProto(Template* temp);

		/**
		convert a proto to template, the caller is responsible for freeing the object
		*/
		Template* ConvertToTemplate(const proto::TemplateSet& temp);

		void AddGaitInfoToKinChain(const proto::TemplateSet& input, KinChain * kinchain);
		void AddGaitInfoToProto(KinChain* kinChain, proto::TemplateSet* proto) ;

	};

}