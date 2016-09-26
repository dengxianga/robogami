#ifndef __ATOM__
#define __ATOM__

#include <vector>
#include <string>
#include <stdio.h>
#include <map>

/*
Atom - A simple hash table mapping a unique string to a unique
number. This reduces string operation and used in symbol table and
other places.
*/
class Atom {

public:
	Atom() {
		// Initialize the table
		stringTable.reserve( 1024);

		hash_table.resize(stringTable.size() * 2+16);
		for(unsigned int i=0;i< hash_table.size();i++) {
			hash_table[i].first = -1;
		}
		//amap.resize(1024);
		//arev.resize(1024);
	}

	// Return atom number of the string
	int LookUpAddString( const std::string &str){
		int key = find_hash(str);
		int atom = -1;
		// This case should never ever happen
		if (key == -1)
			return - 1;
		if (hash_table[key].first == -1) {

			if (stringTable.size() * 2 > hash_table.size() ) {
				increase_size(); 
			}
			key = find_hash(str);
			atom = (int)stringTable.size();
			stringTable.push_back(str);

			hash_table[key].first = atom;
			hash_table[key].second = hash_function2(str);
		} else
			atom = hash_table[key].first;
		return atom;
	}

	std::string LookUpString( int atom) {
		if ((unsigned int)atom < stringTable.size()) {
			return stringTable[atom];
		}
		return "";
	}

private:
	std::vector< std::string >  stringTable;
	//    std::vector< int  > amap;
	//    std::vector< int  > arev;
	std::vector< std::pair<int, int>  > hash_table;

	void  increase_size() {
		// Always double
		hash_table.resize( (hash_table.size() << 1) + 16);
		for(unsigned int i=0; i < hash_table.size() ;i++) {
			hash_table[i].first = -1;
		}
		// Rehash
		for(unsigned int atom=0; atom < stringTable.size() ;atom++) {
			int key = find_hash(stringTable[atom]);
			if (hash_table[key].first != -1) {
				fprintf(stderr, "ERROR: The hash slot is not empty");
				return;
			}
			hash_table[key].first = atom;
			hash_table[key].second = hash_function2( stringTable[atom] );
		}

	}

	int hash_function( const std::string &str) {
		int key = 0;
		const char * s = str.c_str();
		while (*s) {
			key = (key*13507 + *s*287) ^ (key >> 2);
			s++;
		}
		return key & 0x7fffffff;
	}
	int hash_function2( const std::string &str) {
		int key = 0;
		const char * s = str.c_str();
		while (*s) {
			key = (key*3507 + *s*217) ^ (key >> 1);
			s++;
		}
		return key & 0x7fffffff;
	}
	int find_hash( const std::string &str) {
		int key = hash_function(str)    % hash_table.size();
		int key2 = hash_function2(str) ;
		unsigned int count = 0;
		for(; count < hash_table.size(); count++, key = (key + 1) % hash_table.size()) {
			int ret = hash_table[ key ].first;
			int ret_key2 = hash_table[ key ].second;
			if ( ret == -1 ||
				(ret_key2 == key2 && stringTable[ret] == str)) {
					return key;
			}
		}
		// If it cannot find empty space or the hash result, increase the size.
		increase_size();
		return find_hash(str);
	}
};


#endif