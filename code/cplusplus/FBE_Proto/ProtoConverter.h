#pragma once
#include <google\protobuf\message.h>
#include <google\protobuf\text_format.h>
#include <fstream>
#include <sstream>

namespace FabByExample {
	template<class ProtoClass>
	class ProtoConverter
	{
	public:
		void saveToFile(const ProtoClass& temp, const char* filename, bool ascii = false);
		ProtoClass* loadFromFile(const char* filename, bool ascii = true);
	};

	template <class ProtoClass>
	void ProtoConverter<ProtoClass>::saveToFile(ProtoClass const& proto, char const* filename, bool ascii) {
		if (ascii) {
			std::string output;
			google::protobuf::TextFormat::PrintToString(proto, &output);
			std::ofstream outfile(std::string(filename), std::ios::out);
			outfile << output;
			outfile.close();
		}
		else {
			std::ofstream outfile(std::string(filename), std::ios::out | std::ios::binary);
			proto.SerializePartialToOstream(&outfile);
			outfile.close();
		}
	}

	template <class ProtoClass>
	ProtoClass* ProtoConverter<ProtoClass>::loadFromFile(char const* filename, bool ascii) {
		if (ascii) {
			std::ostringstream input;
			std::ifstream infile(std::string(filename), std::ios::in);
			input << infile.rdbuf();
			std::string inputStr = input.str();
			ProtoClass* result = new ProtoClass();
			bool success = google::protobuf::TextFormat::ParseFromString(inputStr, result);
			return result;
		}
		else {
			std::ifstream infile(std::string(filename), std::ios::in | std::ios::binary);
			ProtoClass* loadedProto = new ProtoClass();
			bool success = loadedProto->ParseFromIstream(&infile);
			//assert(success);
			if(!success)
				return nullptr;
			return loadedProto;
		}
	}
}