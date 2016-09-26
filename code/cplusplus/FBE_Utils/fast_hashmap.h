#include <unordered_map>

/*
WARNING: DO NOT INCLUDE THIS FILE UNTIL YOU READ THE FOLLOWING INSTRUCTIONS.

Use with the following instructions. Suppose you want a fast_hashmap named KVMap of key = type K, value = type V, then
	1. Create a KVMap.h file, with the following contents:
	    #pragma once
		#define I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP
		#define FAST_HASHMAP_HEADERS
		namespace _KVMapImpl {
			typedef K Key;
			typedef V Value;
			#include "fast_hashmap.h"
		}
		typedef _KVMapImpl::fast_hashmap KVMap
		#undef FAST_HASHMAP_HEADERS
		#undef I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP
	
	2. Create a KVMap.cpp file, with the following contents:
		#include "KVMap.h"
		#define I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP
		#define FAST_HASHMAP_IMPL
		namespace _KVMapImpl {
			#include "fast_hashmap.h"
		}
		#undef FAST_HASHMAP_IMPL
		#undef I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP
*/

//#undef I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP

#ifndef I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP
#pragma error "Don't include this until you know how to."
#endif


#ifndef I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP
typedef int Key;
typedef int Value;
#endif


#if defined FAST_HASHMAP_HEADERS || !defined I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP

const int fast_hashmap_static_limit = 3;
struct fast_hashmap;

struct map_placeholder {
	char placeholder[sizeof(std::unordered_map<Key, Value>)];
	std::unordered_map<Key, Value>* asMap() {
		return (std::unordered_map<Key, Value>*)(void*)(&placeholder);
	}
	const std::unordered_map<Key, Value>* asMap() const {
		return (const std::unordered_map<Key, Value>*)(const void*)(void*)(&placeholder);
	}
};

struct map_iter_placeholder {
	char placeholder[sizeof(std::unordered_map<Key, Value>::iterator)];
	std::unordered_map<Key, Value>::iterator& asIter() {
		return *(std::unordered_map<Key, Value>::iterator*)(void*)(&placeholder);
	}
	const std::unordered_map<Key, Value>::iterator& asIter() const {
		return *(const std::unordered_map<Key, Value>::iterator*)(void*)(&placeholder);
	}
};

struct map_const_iter_placeholder {
	char placeholder[sizeof(std::unordered_map<Key, Value>::const_iterator)];
	std::unordered_map<Key, Value>::const_iterator& asIter() {
		return *(std::unordered_map<Key, Value>::const_iterator*)(void*)(&placeholder);
	}
	const std::unordered_map<Key, Value>::const_iterator& asIter() const {
		return *(const std::unordered_map<Key, Value>::const_iterator*)(void*)(&placeholder);
	}
};

struct fast_hashmap_const_iterator {
	const fast_hashmap* map;
	union {
		int index;
		map_const_iter_placeholder map_iter;
	};
	fast_hashmap_const_iterator() : map(nullptr) {}
	fast_hashmap_const_iterator(fast_hashmap_const_iterator const&);
	const std::pair<const Key, Value>& operator*() const;
	const std::pair<const Key, Value>* operator->() const;
	fast_hashmap_const_iterator& operator++();
	bool operator!=(const fast_hashmap_const_iterator& other) const;
	~fast_hashmap_const_iterator();
private:
	void operator=(fast_hashmap_const_iterator const& other) { throw "Do not use"; }
};

struct fast_hashmap_iterator {
	fast_hashmap* map;
	union {
		int index;
		map_iter_placeholder map_iter;
	};
	fast_hashmap_iterator() : map(nullptr) {}
	fast_hashmap_iterator(fast_hashmap_iterator const&);
	std::pair<const Key, Value>& operator*() const;
	std::pair<const Key, Value>* operator->() const;
	fast_hashmap_iterator& operator++();
	bool operator!=(const fast_hashmap_iterator& other) const;
	operator fast_hashmap_const_iterator();
	~fast_hashmap_iterator();
private:
	void operator=(fast_hashmap_iterator const& other) { throw "Do not use"; }
};


struct fhvtable {
	virtual void put(fast_hashmap* map, const Key&, const Value&) = 0;
	virtual const Value* get(const fast_hashmap* map, const Key&) = 0;
	virtual Value* get(fast_hashmap* map, const Key&) = 0;
	virtual int size(const fast_hashmap* map) = 0;
	virtual void free(const fast_hashmap* map) = 0;
	virtual fast_hashmap_iterator begin(fast_hashmap* map) = 0;
	virtual fast_hashmap_const_iterator begin(const fast_hashmap* map) = 0;
	virtual fast_hashmap_iterator end(fast_hashmap* map) = 0;
	virtual fast_hashmap_const_iterator end(const fast_hashmap* map) = 0;
	virtual void next(fast_hashmap_iterator& iter) = 0;
	virtual void next(fast_hashmap_const_iterator& iter) = 0;
	virtual std::pair<const Key, Value>& deref(const fast_hashmap_iterator& iter) = 0;
	virtual const std::pair<const Key, Value>& deref(const fast_hashmap_const_iterator& iter) = 0;
	virtual bool iter_eq(const fast_hashmap_iterator&, const fast_hashmap_iterator&) = 0;
	virtual bool iter_eq(const fast_hashmap_const_iterator&, const fast_hashmap_const_iterator&) = 0;
	virtual void iter_free(const fast_hashmap_iterator&) = 0;
	virtual void iter_free(const fast_hashmap_const_iterator&) = 0;
};

struct fhvt_inline : public fhvtable {
	virtual void put(fast_hashmap* map, const Key& key, const Value& value);
	virtual const Value* get(const fast_hashmap* map, const Key&);
	virtual Value* get(fast_hashmap* map, const Key&);
	virtual int size(const fast_hashmap* map);
	virtual void free(const fast_hashmap* map);
	virtual fast_hashmap_iterator begin(fast_hashmap* map);
	virtual fast_hashmap_const_iterator begin(const fast_hashmap* map);
	virtual fast_hashmap_iterator end(fast_hashmap* map);
	virtual fast_hashmap_const_iterator end(const fast_hashmap* map);
	virtual void next(fast_hashmap_iterator& iter);
	virtual void next(fast_hashmap_const_iterator& iter);
	virtual std::pair<const Key, Value>& deref(const fast_hashmap_iterator& iter);
	virtual const std::pair<const Key, Value>& deref(const fast_hashmap_const_iterator& iter);
	virtual bool iter_eq(const fast_hashmap_iterator&, const fast_hashmap_iterator&);
	virtual bool iter_eq(const fast_hashmap_const_iterator&, const fast_hashmap_const_iterator&);
	virtual void iter_free(const fast_hashmap_iterator&);
	virtual void iter_free(const fast_hashmap_const_iterator&);
	static fhvt_inline value;
private:
	Value* fhvt_inline::internal_get(fast_hashmap* map, const Key& key);
};

struct fhvt_map : public fhvtable {
	virtual void put(fast_hashmap* map, const Key&, const Value&);
	virtual const Value* get(const fast_hashmap* map, const Key&);
	virtual Value* get(fast_hashmap* map, const Key&);
	virtual int size(const fast_hashmap* map);
	virtual void free(const fast_hashmap* map);
	virtual fast_hashmap_iterator begin(fast_hashmap* map);
	virtual fast_hashmap_const_iterator begin(const fast_hashmap* map);
	virtual fast_hashmap_iterator end(fast_hashmap* map);
	virtual fast_hashmap_const_iterator end(const fast_hashmap* map);
	virtual void next(fast_hashmap_iterator& iter);
	virtual void next(fast_hashmap_const_iterator& iter);
	virtual std::pair<const Key, Value>& deref(const fast_hashmap_iterator& iter);
	virtual const std::pair<const Key, Value>& deref(const fast_hashmap_const_iterator& iter);
	virtual bool iter_eq(const fast_hashmap_iterator&, const fast_hashmap_iterator&);
	virtual bool iter_eq(const fast_hashmap_const_iterator&, const fast_hashmap_const_iterator&);
	virtual void iter_free(const fast_hashmap_iterator&);
	virtual void iter_free(const fast_hashmap_const_iterator&);
	static fhvt_map value;
};



struct fast_hashmap {
private:
	fhvtable* funcs;
	union {
		struct {
			int count;
			std::pair<Key, Value> inline_items[fast_hashmap_static_limit];
		};
		map_placeholder map;
	};

public:
	fast_hashmap() : count(), funcs(&(fhvt_inline::value)) {
	}
	fast_hashmap(const fast_hashmap&);

	fast_hashmap& operator=(const fast_hashmap&);

	~fast_hashmap() {
		funcs->free(this);
	}
	typedef fast_hashmap_iterator iterator;
	typedef fast_hashmap_const_iterator const_iterator;
	void put(const Key& key, const Value& value) {
		funcs->put(this, key, value);
	}

	const Value& at(const Key& key) const {
		return *funcs->get(this, key);
	}

	bool has(const Key& key) const {
		return funcs->get(this, key) != nullptr;
	}

	Value& operator[](const Key& key) {
		Value* v = funcs->get(this, key);
		if (v != nullptr) {
			return *v;
		}
		funcs->put(this, key, Value());
		return *funcs->get(this, key);
	}

	int size() const {
		return funcs->size(this);
	}
	
	bool empty() const {
		return funcs->size(this) == 0;
	}

	iterator begin() {
		return funcs->begin(this);
	}

	iterator end() {
		return funcs->end(this);
	}

	const_iterator begin() const {
		return funcs->begin(this);
	}

	const_iterator end() const {
		return funcs->end(this);
	}
	
	bool operator==(const fast_hashmap& other) const;

	friend class fhvt_inline;
	friend class fhvt_map;
	friend class fast_hashmap_iterator;
	friend class fast_hashmap_const_iterator;
};

#endif

#if defined FAST_HASHMAP_IMPL || !defined I_KNOW_HOW_TO_INCLUDE_FAST_HASHMAP


std::pair<const Key, Value>& fast_hashmap_iterator::operator*() const {
	return map->funcs->deref(*this);
}
std::pair<const Key, Value>* fast_hashmap_iterator::operator->() const {
	return &map->funcs->deref(*this);
}
fast_hashmap_iterator& fast_hashmap_iterator::operator++() {
	map->funcs->next(*this);
	return *this;
}
bool fast_hashmap_iterator::operator!=(const fast_hashmap_iterator& other) const {
	return !map->funcs->iter_eq(*this, other);
}


const std::pair<const Key, Value>& fast_hashmap_const_iterator::operator*() const {
	return map->funcs->deref(*this);
}
const std::pair<const Key, Value>* fast_hashmap_const_iterator::operator->() const {
	return &map->funcs->deref(*this);
}
fast_hashmap_const_iterator& fast_hashmap_const_iterator::operator++() {
	map->funcs->next(*this);
	return *this;
}
bool fast_hashmap_const_iterator::operator != (const fast_hashmap_const_iterator& other) const {
	return !map->funcs->iter_eq(*this, other);
}








void fhvt_inline::put(fast_hashmap* map, const Key& key, const Value& value) {
	Value* existing = internal_get(map, key);
	if (existing) {
		*existing = value;
	}
	else if (map->count < fast_hashmap_static_limit) {
		auto& newPair = map->inline_items[map->count];
		newPair.first = key;
		newPair.second = value;
		++map->count;
	}
	else {
		std::pair<Key, Value> inline_items_copy[fast_hashmap_static_limit];
		for (int i = 0; i < fast_hashmap_static_limit; i++) {
			inline_items_copy[i] = map->inline_items[i];
		}
		new(map->map.asMap())std::unordered_map<Key, Value>();
		for (int i = 0; i < fast_hashmap_static_limit; i++) {
			map->map.asMap()->insert(inline_items_copy[i]);
		}
		map->map.asMap()->insert(std::make_pair(key, value));
		map->funcs = &(fhvt_map::value);
	}
}
const Value* fhvt_inline::get(const fast_hashmap* map, const Key& key) {
	return internal_get(const_cast<fast_hashmap*>(map), key);
}
Value* fhvt_inline::get(fast_hashmap* map, const Key& key) {
	return internal_get(map, key);
}
Value* fhvt_inline::internal_get(fast_hashmap* map, const Key& key) {
	for (int i = 0; i < map->count; i++) {
		if (map->inline_items[i].first == key) {
			return &map->inline_items[i].second;
		}
	}
	return nullptr;
}

void fhvt_map::put(fast_hashmap* map, const Key& key, const Value& value) {
	map->map.asMap()->insert(std::make_pair(key, value));
}
const Value* fhvt_map::get(const fast_hashmap* map, const Key& key) {
	auto it = map->map.asMap()->find(key);
	if (it == map->map.asMap()->end()) {
		return nullptr;
	}
	return &it->second;
}
Value* fhvt_map::get(fast_hashmap* map, const Key& key) {
	auto it = map->map.asMap()->find(key);
	if (it == map->map.asMap()->end()) {
		return nullptr;
	}
	return &it->second;
}
int fhvt_inline::size(const fast_hashmap* map) {
	return map->count;
}
int fhvt_map::size(const fast_hashmap* map) {
	return map->map.asMap()->size();
}

void fhvt_inline::free(const fast_hashmap* map) {
	
}
void fhvt_map::free(const fast_hashmap* map) {
	map->map.asMap()->~unordered_map<Key, Value>();
}


fast_hashmap_iterator fhvt_inline::begin(fast_hashmap* map) {
	fast_hashmap_iterator it;
	it.map = map;
	it.index = 0;
	return it;
}
fast_hashmap_iterator fhvt_inline::end(fast_hashmap* map) {
	fast_hashmap_iterator it;
	it.map = map;
	it.index = map->count;
	return it;
}
void fhvt_inline::next(fast_hashmap_iterator& iter) {
	++iter.index;
}
std::pair<const Key, Value>& fhvt_inline::deref(const fast_hashmap_iterator& iter) {
	return reinterpret_cast<std::pair<const Key, Value>&>(iter.map->inline_items[iter.index]);
}

fast_hashmap_iterator fhvt_map::begin(fast_hashmap* map) {
	fast_hashmap_iterator it;
	it.map = map;
	new (&it.map_iter.asIter())std::unordered_map<Key, Value>::iterator(map->map.asMap()->begin());
	return it;
}
fast_hashmap_iterator fhvt_map::end(fast_hashmap* map) {
	fast_hashmap_iterator it;
	it.map = map;
	new (&it.map_iter.asIter())std::unordered_map<Key, Value>::iterator(map->map.asMap()->end());
	return it;
}
void fhvt_map::next(fast_hashmap_iterator& iter) {
	++iter.map_iter.asIter();
}
std::pair<const Key, Value>& fhvt_map::deref(const fast_hashmap_iterator& iter) {
	return *iter.map_iter.asIter();
}


bool fhvt_inline::iter_eq(const fast_hashmap_iterator& a, const fast_hashmap_iterator& b) {
	return a.index == b.index;
}
bool fhvt_map::iter_eq(const fast_hashmap_iterator& a, const fast_hashmap_iterator& b) {
	return a.map_iter.asIter() == b.map_iter.asIter();
}
void fhvt_inline::iter_free(const fast_hashmap_iterator& it) {
}
void fhvt_map::iter_free(const fast_hashmap_iterator& it) {
	it.map_iter.asIter().~_List_iterator();
}



fast_hashmap_const_iterator fhvt_inline::begin(const fast_hashmap* map) {
	fast_hashmap_const_iterator it;
	it.map = map;
	it.index = 0;
	return it;
}
fast_hashmap_const_iterator fhvt_inline::end(const fast_hashmap* map) {
	fast_hashmap_const_iterator it;
	it.map = map;
	it.index = map->count;
	return it;
}
void fhvt_inline::next(fast_hashmap_const_iterator& iter) {
	++iter.index;
}
const std::pair<const Key, Value>& fhvt_inline::deref(const fast_hashmap_const_iterator& iter) {
	return reinterpret_cast<const std::pair<const Key, Value>&>(iter.map->inline_items[iter.index]);
}

fast_hashmap_const_iterator fhvt_map::begin(const fast_hashmap* map) {
	fast_hashmap_const_iterator it;
	it.map = map;
	new (&it.map_iter.asIter())std::unordered_map<Key, Value>::const_iterator(map->map.asMap()->begin());
	return it;
}
fast_hashmap_const_iterator fhvt_map::end(const fast_hashmap* map) {
	fast_hashmap_const_iterator it;
	it.map = map;
	new (&it.map_iter.asIter())std::unordered_map<Key, Value>::const_iterator(map->map.asMap()->end());
	return it;
}
void fhvt_map::next(fast_hashmap_const_iterator& iter) {
	++iter.map_iter.asIter();
}
const std::pair<const Key, Value>& fhvt_map::deref(const fast_hashmap_const_iterator& iter) {
	return *iter.map_iter.asIter();
}


bool fhvt_inline::iter_eq(const fast_hashmap_const_iterator& a, const fast_hashmap_const_iterator& b) {
	return a.index == b.index;
}
bool fhvt_map::iter_eq(const fast_hashmap_const_iterator& a, const fast_hashmap_const_iterator& b) {
	return a.map_iter.asIter() == b.map_iter.asIter();
}
void fhvt_inline::iter_free(const fast_hashmap_const_iterator& it) {
}
void fhvt_map::iter_free(const fast_hashmap_const_iterator& it) {
	it.map_iter.asIter().~_List_const_iterator();
}

fast_hashmap_iterator::fast_hashmap_iterator(fast_hashmap_iterator const& other) {
	if (other.map == nullptr || other.map->funcs == &fhvt_inline::value) {
		memcpy(this, &other, sizeof(fast_hashmap_iterator));
	}
	else {
		map = other.map;
		new (&this->map_iter.asIter())std::unordered_map<Key, Value>::iterator(other.map_iter.asIter());
	}
}

fast_hashmap_const_iterator::fast_hashmap_const_iterator(fast_hashmap_const_iterator const& other) {
	if (other.map == nullptr || other.map->funcs == &fhvt_inline::value) {
		memcpy(this, &other, sizeof(fast_hashmap_const_iterator));
	}
	else {
		map = other.map;
		new (&this->map_iter.asIter())std::unordered_map<Key, Value>::const_iterator(other.map_iter.asIter());
	}
}

fast_hashmap_iterator::operator fast_hashmap_const_iterator() {
	return *reinterpret_cast<fast_hashmap_const_iterator*>(this);
}

fast_hashmap_const_iterator::~fast_hashmap_const_iterator() {
	map->funcs->iter_free(*this);
}
fast_hashmap_iterator::~fast_hashmap_iterator() {
	map->funcs->iter_free(*this);
}



fhvt_inline fhvt_inline::value;
fhvt_map fhvt_map::value;


fast_hashmap::fast_hashmap(const fast_hashmap& from) {
	if (from.funcs == &fhvt_inline::value) {
		memcpy(this, &from, sizeof(fast_hashmap));
	}
	else {
		new (map.asMap())std::unordered_map<Key, Value>(*from.map.asMap());
		funcs = from.funcs;
	}
}

fast_hashmap& fast_hashmap::operator=(const fast_hashmap& from) {
	bool thisInline = funcs == &fhvt_inline::value;
	bool otherInline = from.funcs == &fhvt_inline::value;
	if (thisInline && otherInline) {
		memcpy(this, &from, sizeof(fast_hashmap));
	}
	else if (thisInline) {
		new (map.asMap())std::unordered_map<Key, Value>(*from.map.asMap());
		funcs = from.funcs;
	}
	else if (otherInline) {
		map.asMap()->~unordered_map();
		memcpy(this, &from, sizeof(fast_hashmap));
	}
	else {
		*map.asMap() = *from.map.asMap();
	}
	return *this;
}

bool fast_hashmap::operator==(const fast_hashmap& other) const {
	if (size() != other.size()) return false;
	for each (const auto& pair in *this) {
		if (!(other.has(pair.first) && other.at(pair.first) == pair.second)) {
			return false;
		}
	}
	for each (const auto& pair in other) {
		if (!(has(pair.first) && at(pair.first) == pair.second)) {
			return false;
		}
	}
	return true;
}

#endif