#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <limits>
#include <math.h>
#include <vector>

template<class T>
class MinHeap
{
public:
	MinHeap(int InitialSize);
	~MinHeap();
	
	void Add(T Item, int Value);
	bool GetMin(T& Item);
	
private:
	typedef struct {
		T Item;
		int Value;
	} Node;
	
	Node* Elements;
	int  MaxElements;
	int  NumElements;
	
	Node* GetLeftChild(Node* node)
	{
		int index = node - Elements;
		int LeftChild = 2 * index + 1;
		if (LeftChild >= NumElements) return NULL;
		return &Elements[LeftChild];
	}
	Node* GetRightChild(Node* node)
	{
		int index = node - Elements;
		int RightChild = 2 * index + 2;
		if (RightChild >= NumElements) return NULL;
		return &Elements[RightChild];
	}
	Node* GetParent(Node* node)
	{
		int index = node - Elements;
		if (index <= 0 ) return NULL;
		int Parent = (index - 1) / 2;
		return &Elements[Parent];
	}
	void Swap(Node* node1, Node* node2)
	{
		Node tmp = *node1;
		*node1 = *node2;
		*node2 = tmp;
	}
	bool IsNodeLeaf(Node* node)
	{
		return (GetRightChild(node) == NULL && GetLeftChild(node) == NULL);
	}
	bool IsNodeFull(Node* node)
	{
		return (GetRightChild(node) != NULL && GetLeftChild(node) != NULL );
	}
};

template<class T>
MinHeap<T>::MinHeap(int InitialSize)
{
	MaxElements = InitialSize;
	Elements = new Node[MaxElements];
	NumElements = 0;
}

template<class T>
MinHeap<T>::~MinHeap()
{
	delete Elements;
}

template<class T>
void MinHeap<T>::Add(T Item, int Value)
{
	assert(NumElements <= MaxElements);
	if (NumElements == MaxElements) {
		// double the storage size
		int newMaxElements = MaxElements*2;
		Node* newStorage = new Node[newMaxElements];
		for(int i=0; i<MaxElements; i++) {
			newStorage[i] = Elements[i];
		}
		delete Elements;
		Elements = newStorage;
		MaxElements = newMaxElements;
	}
	assert(NumElements < MaxElements);
	// Add item to end of heap
	Elements[NumElements].Value = Value;
	Elements[NumElements].Item = Item;
	// restore heap structure by sifting up
	Node* node = &Elements[NumElements];
	NumElements++;
	Node* parent = GetParent(node);
	while (parent != NULL && parent->Value > node->Value) {
		// sift up, swap elements
		Swap(node, parent);
		node = parent;
		parent = GetParent(node);
	}
}

template<class T>
bool MinHeap<T>::GetMin(T& Item)
{
	if (NumElements == 0) return false;
	
	// Min Value is at the top of the heap
	Item = Elements[0].Item;
	
	if (NumElements == 1) {
		NumElements--;
		return true;
	}
	
	// Now place the last item in the first spot and sift-down until heap is restored
	Elements[0] = Elements[NumElements-1];
	NumElements--;
	
	Node* node = &Elements[0];
	Node* leftChild = GetLeftChild(node);
	Node* rightChild = GetRightChild(node);
	while (1) {
		if (IsNodeLeaf(node)) break;
		if ( (leftChild == NULL && rightChild != NULL) || (IsNodeFull(node) && rightChild->Value <= leftChild->Value) ) {
			if (rightChild->Value < node->Value) {
				Swap(rightChild, node);
				node = rightChild;
			}
			else break;
		}
		else if ( (rightChild == NULL && leftChild != NULL) || (IsNodeFull(node) && leftChild->Value <= rightChild->Value) ) {
			if (leftChild->Value < node->Value) {
				Swap(leftChild, node);
				node = leftChild;
			}
			else break;
		}
		
		leftChild = GetLeftChild(node);
		rightChild = GetRightChild(node);
	}
	return true;
}

class Teleporter
{
public:
	Teleporter() {
	}
	Teleporter(int energy, int xPos, int yPos) {
		mX = xPos;
		mY = yPos;
		mEnergyRequired = energy;
	}
	int energyRequired() const {
		return mEnergyRequired;
	}
	int x() const {
		return mX;
	}
	int y() const {
		return mY;
	}
private:
	int mEnergyRequired;
	int mX;
	int mY;
};

class ITeleporterController
{
public:
	virtual ~ITeleporterController() {}
	virtual void initialize(std::vector<Teleporter> teleporters) = 0;
	virtual bool findPath(int startTeleporter, int endTeleporter, int clientsEnergy, 
						  int maxTeleportationDist, std::vector<int>& outPath) = 0;
};

#define MAX_INT std::numeric_limits<int>::max()

class MyTeleporterController : public ITeleporterController
{
public:
	~MyTeleporterController()
	{
		delete mPriorityQueue;
	}
	
	void initialize(std::vector<Teleporter> teleporters)
	{
		mPriorityQueue = new MinHeap<int>(teleporters.size());
		
		mTeleporters.reserve(teleporters.size());
		for (int i=0; i<teleporters.size(); i++) {
			TeleporterInfo info;
			info.teleporter = teleporters[i];
			info.visited = false;
			info.cost = MAX_INT;
			info.previous = -1;
			mTeleporters.push_back(info);
		}
	}
	
	bool findPath(int startTeleporter, int endTeleporter, int clientsEnergy, 
						  int maxTeleportationDist, std::vector<int>& outPath)
	{
		mPriorityQueue->Add(startTeleporter, 0);
		mTeleporters[startTeleporter].cost = 0;
		int maxTeleportationDistSq = maxTeleportationDist * maxTeleportationDist;
		
		int current;
		bool pathFound = false;
		while ( mPriorityQueue->GetMin(current) ) {
			if (mTeleporters[current].visited) {
				continue;
			}
			
			if (current == endTeleporter) {
				pathFound = true;
				break;
			}
			// find neighbors of current that are within teleportation range
			for (int i=0; i<mTeleporters.size(); i++) {
				if (i == current) continue;  // skip self
				if (mTeleporters[i].visited) continue;
				int distance = Dist(&mTeleporters[current].teleporter, &mTeleporters[i].teleporter);
				if (distance <= maxTeleportationDistSq) {
					int newCost = mTeleporters[current].cost + mTeleporters[current].teleporter.energyRequired();
					if (newCost < mTeleporters[i].cost && newCost < clientsEnergy) {
						mTeleporters[i].cost = newCost;
						mTeleporters[i].previous = current;
						mPriorityQueue->Add(i, newCost);
					}
				}
			}
			mTeleporters[current].visited = true;
		}
		
		if (pathFound) {
			// fill in the shortest path
			outPath.push_back(endTeleporter);
			int i = endTeleporter;
			while (mTeleporters[i].previous != -1) {
				outPath.insert(outPath.begin(), mTeleporters[i].previous);
				i = mTeleporters[i].previous;
			}
			return true;
		}
		return false;
	}
	
private:
	struct TeleporterInfo
	{
		Teleporter teleporter;
		bool visited;
		int cost;
		int previous;
	};
	
	MinHeap<int>* mPriorityQueue;
	std::vector<TeleporterInfo> mTeleporters;
					
	int Dist(Teleporter* t1, Teleporter* t2)
	{
		int dx = abs(t1->x() - t2->x());
		int dy = abs(t1->y() - t2->y());
		return (dx * dx + dy * dy);
	}
};

int main (int argc, char * const argv[]) {
	
	std::cout << "start searching" << std::endl;
	
	for (int j=0; j<100; j++)
	{
		srand( time(NULL) + j );
		
		// generate a bunch of test teleporters
		const int NumTeleporters = 25;
		std::vector<Teleporter> teleporters;
		teleporters.reserve(NumTeleporters);
		for (int i=0; i<NumTeleporters; i++) {
			teleporters.push_back(Teleporter(rand() % 10, rand() % 100, rand() % 100));
		}
		MyTeleporterController c;
		int startTeleporter = 0;
		int endTeleporter = NumTeleporters - 1;
		int energy = 40;
		int maxTeleportationDist = 30;
		std::vector<int> outPath;
		c.initialize(teleporters);
		if (c.findPath(startTeleporter, endTeleporter, energy, maxTeleportationDist, outPath)) {
			std::cout << "Found a path: ";
			for (int i=0; i<outPath.size(); i++) {
				std::cout << outPath[i] << ", ";
			}
			std::cout << std::endl;
		}
		else {
			std::cout << "Did not find a path" << std::endl;
		}
	}
		
    return 0;
}
