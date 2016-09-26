#include <cstdio>
#include <cstdlib>
#include <vector>
//using namespace std;


class VHashTable {
public:
	typedef Vec<3,int> voxel_coord_t;
	typedef size_t key_t;
	typedef size_t data_t;
	static const size_t MAGIC1 = 100000007;
	static const size_t MAGIC2 = 161803409;
	static const size_t MAGIC3 = 423606823;
	static const size_t SIZE_FUDGE = 2;
	static const data_t NO_DATA = 0xffffffffu;

private:
	float voxelsize, scale;
	std::vector<voxel_coord_t> voxel_coords;
	std::vector<data_t> data;

public:
	VHashTable(size_t maxpoints, float voxelsize_) :
		voxelsize(voxelsize_), scale(1.0f / voxelsize_)
	{
		size_t n = SIZE_FUDGE * maxpoints;
		voxel_coords.resize(n);
		// Work around a bizarre compiler bug
		data_t tmp = NO_DATA;
		data.resize(n, tmp);
	}
	data_t &operator[] (const point &p)
	{
		voxel_coord_t c(int(floor(p[0] * scale)),
		                int(floor(p[1] * scale)),
				int(floor(p[2] * scale)));
		key_t key = MAGIC1 * c[0] + MAGIC2 * c[1] + MAGIC3 * c[2];
		key %= data.size();

		// Open hashing
		while (1) {
			if (data[key] == NO_DATA) {
				voxel_coords[key] = c;
				break;
			} else if (voxel_coords[key] == c) {
				break;
			}
			key++;
			if (key == data.size())
				key = 0;
		}

		return data[key];
	}
};
