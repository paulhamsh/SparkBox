#ifndef CircularArray_h
#define CircularArray_h

#define ARRAY_SIZE 1500

class CircularArray
{ 
  public:
  uint8_t buf[ARRAY_SIZE];
  int size; 
  int start; 
  int end;

  CircularArray();
  int length();
  uint8_t& operator[](int index);
  int expand (int len);
  int shrink(int len);
  //int copy_in(uint8_t *data, int index, int len);
  //int copy_out(uint8_t *data, int index, int len);
  int append(uint8_t *data, int len);
  int extract(uint8_t *data, int len_to_copy, int len_to_shrink);
  int extract_append(CircularArray &to, int len_to_copy, int len_to_shrink);
  void show();
  void clear();
};



#endif