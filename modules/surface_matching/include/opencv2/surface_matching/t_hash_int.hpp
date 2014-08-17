
#ifndef __OPENCV_T_HASH_INT_HPP__
#define __OPENCV_T_HASH_INT_HPP__

#include <stdio.h>
#include <stdlib.h>

namespace cv 
{
	namespace ppf_match_3d 
	{

		typedef unsigned int KeyType;

		typedef struct hashnode_i {
			KeyType key;
			void *data;
			struct hashnode_i *next;
		} hashnode_i ;

		typedef struct HSHTBL_i {
			size_t size;
			struct hashnode_i **nodes;
			size_t (*hashfunc)(unsigned int);
		} hashtable_int;


		__inline static unsigned int next_power_of_two(unsigned int value)
		{
			/* Round up to the next highest power of 2 */
			/* from http://www-graphics.stanford.edu/~seander/bithacks.html */

			--value;
			value |= value >> 1;
			value |= value >> 2;
			value |= value >> 4;
			value |= value >> 8;
			value |= value >> 16;
			++value;

			return value;
		}

		hashtable_int *hashtable_int_create(size_t size, size_t (*hashfunc)(unsigned int));
		void hashtable_int_destroy(hashtable_int *hashtbl);
		unsigned int hashtable_int_insert(hashtable_int *hashtbl, KeyType key, void *data);
		unsigned int hashtable_int_insert_hashed(hashtable_int *hashtbl, KeyType key, void *data);
		unsigned int hashtable_int_remove(hashtable_int *hashtbl, KeyType key);
		void *hashtable_int_get(hashtable_int *hashtbl, KeyType key);
		hashnode_i* hashtable_int_get_bucket_hashed(hashtable_int *hashtbl, KeyType key);
		unsigned int hashtable_int_resize(hashtable_int *hashtbl, size_t size);
		hashtable_int *hashtable_int_clone(hashtable_int *hashtbl);
		hashtable_int *hashtable_int_read(FILE* f);
		int hashtable_int_write(const hashtable_int * hashtbl, const size_t dataSize, FILE* f);
		void hashtable_int_print(hashtable_int *hashtbl);


	}
}
#endif

