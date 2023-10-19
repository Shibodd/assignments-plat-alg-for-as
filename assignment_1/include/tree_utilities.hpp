/* \author Aaron Brown */

#include "Renderer.hpp"
#include <Eigen/Geometry>

namespace my_kdtree
{

constexpr int DIMENSIONS = 3;
// Structure to represent node of kd tree

struct Node
{
	Node* parent;
	std::unique_ptr<Node> left;
	std::unique_ptr<Node> right;
	Eigen::Vector3f point;
	int id;

	Node(Node* parent, Eigen::Vector3f point, int id)
	:	parent(parent), point(point), id(id), left(nullptr), right(nullptr)
	{}
};


class KdTree
{
	inline std::unique_ptr<Node> create_node(Node* parent, Eigen::Vector3f point, int id) {
		return std::unique_ptr<Node>(new Node(parent, point, id));
	}

	void insert(Node* parent, Eigen::Vector3f point, int id, int depth) {
		int index = get_split_by_index(depth);

		while (true) {
			if (point[index] <= parent->point[index]) {
				// Left side
				if (parent->left == nullptr) {
					parent->left = create_node(parent, point, id);
					return;
				} else {
					parent = parent->left.get();
				}
			}
			else {
				// Right side
				if (parent->right == nullptr) {
					parent->right = create_node(parent, point, id);
					return;
				} else {
					parent = parent->right.get();
				}
			}
		}
	}

public:
	std::unique_ptr<Node> root;
	KdTree() : root(nullptr) {}

	constexpr int get_split_by_index(int depth) const
	{
		return (depth % DIMENSIONS);
	}

	inline void insert(Eigen::Vector3f point, int id)
	{	
		if (root == nullptr)
			root = create_node(nullptr, point, id);
		else
			insert(root.get(), point, id, 0);
	}
};
}
