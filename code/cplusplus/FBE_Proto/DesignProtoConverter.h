#pragma once
#include "ProtoConverter.h"
#include "design.pb.h"

namespace FabByExample {
	class Design;

	class DesignProtoConverter : public ProtoConverter<proto::Design> {
	public:
		DesignProtoConverter();
		~DesignProtoConverter();


		/**
		convert a desgin to proto, the caller is responsible for freeing the object
		*/
		proto::Design* ConvertToProto(Design* design);

		/**
		convert a proto to design, the caller is responsible for freeing the object
		*/
		Design* ConvertToDesign(proto::Design*);
	};

}