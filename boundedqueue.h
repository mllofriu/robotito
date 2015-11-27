template<typename Data>
class BoundedQueue {
  size_t d_size; // Stores no. of actually stored objects
  size_t d_capacity; // Stores allocated capacity
  int d_current;
  int d_first;
  float d_sum;
  Data *d_data; // Stores data
  public:
    BoundedQueue(size_t capacity) {
      d_capacity = capacity;
      d_data = new Data[d_capacity];
      d_current = 0;
      d_first = 0;
      d_sum = 0;
      d_size = 0;
    }; // Default constructor
    BoundedQueue(BoundedQueue const &other) : d_current(other.d_current), d_first(other.d_first), d_sum(other.d_sum), d_size(other.d_size), d_capacity(other.d_capacity), d_data(0) {
      d_data = new Data[d_capacity];
      memcpy(d_data, other.d_data, d_size*sizeof(Data)); 
    }; // Copy constuctor
    ~BoundedQueue() { 
      free(d_data);
    }; // Destructor
    BoundedQueue &operator=(BoundedQueue const &other) { 
      free(d_data);
      d_size = other.d_size;
      d_capacity = other.d_capacity; 
      d_current = other.d_current; 
      d_first = other.d_first; 
      d_sum = other.d_sum; 
      d_data = new Data[d_capacity];
      memcpy(d_data, other.d_data, d_size*sizeof(Data));
      return *this;
    }; // Needed for memory management
    void enqueue(Data const &x) {
      if (d_size < d_capacity)
        d_size++; 
      else
        dequeue();

      d_data[d_current] = x;
      // d_current = (d_current + 1) % d_capacity;

      // d_sum += x;
    }; 

    Data &dequeue() {
      int retIdx = d_first;

      d_first = (d_first + 1) % d_capacity;
      d_size--; 

      d_sum -= d_data[retIdx];

      return d_data[retIdx];
    }; 
    size_t size() const { return d_size; }; // Size getter
    float average() { return ((float)d_sum) / d_size; };
    Data const &operator[](size_t idx) const { return d_data[idx]; }; // Const getter
    Data &operator[](size_t idx) { return d_data[idx]; }; // Changeable getter
};
