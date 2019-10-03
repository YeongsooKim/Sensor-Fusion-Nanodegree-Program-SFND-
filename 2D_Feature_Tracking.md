# Task MP.1

Your first task is to set up the loading procedure for the images, which is currently not optimal. In the student version of the code, we push all images into a vector inside a for-loop and with every new image, the data structure grows. Now imagine you want to process a large image sequence with several thousand images and Lidar point clouds over night - in the current implementation this would push the memory of your computer to its limit and eventually slow down the entire program. So in order to prevent this, we only want to hold a certain number of images in memory so that when a new one arrives, the oldest one is deleted from one end of the vector and the new one is added to the other end. The following figure illustrates the principle.

![RingBuffer](https://user-images.githubusercontent.com/51704629/66124679-02184e00-e620-11e9-8213-111e291e51a2.png)

```c++
	// ...modified start: MP.1 Data Buffer Optimization
	// dataBufferSize: no. of images which are held in memory (ring buffer) at the same time
    int dataBufferSize = 3; // default, original code: int dataBufferSize = 2;
	// ...modified end: MP.1 Data Buffer Optimization

  ...
  
  // ...add start: MP.1 Data Buffer Optimization
  if (dataBuffer.size() + 1 > dataBufferSize)
  {
    dataBuffer.erase(dataBuffer.begin());
    cout << "REPLACE IMAGE IN BUFFER done" << endl;
  }
  // ...add end: MP.1 Data Buffer Optimization


```
