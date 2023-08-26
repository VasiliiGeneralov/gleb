#include <cmath>
#include <iostream>
#include <memory>
#include <stack>
#include <string>
#include <unordered_map>
#include <vector>

class Graph {
public:
  Graph() = default;
  void emitDot() const;

  class Edge;

  class Node {
    size_t id_;
    std::string name_;
    std::vector<std::weak_ptr<Edge>> ins_;
    std::vector<std::weak_ptr<Edge>> outs_;

  public:
    Node(size_t id, const std::string &name) {
      id_ = id;
      name_ = name;
    }

    void addOutEdge(std::shared_ptr<Edge> edge) { ins_.emplace_back(edge); }

    void addInEdge(std::shared_ptr<Edge> edge) { outs_.emplace_back(edge); }

    std::string getName() const { return name_; }

    std::vector<std::weak_ptr<Edge>> getOuts() { return outs_; }

    std::vector<std::weak_ptr<Edge>> getIns() { return ins_; }
  };

  class Edge {
    std::weak_ptr<Node> src_;
    std::weak_ptr<Node> dst_;
    double weight_;

  public:
    Edge(std::shared_ptr<Node> src, std::shared_ptr<Node> dst, double weight) {
      src_ = src;
      dst_ = dst;
      weight_ = weight;
    }

    std::weak_ptr<Node> getSrc() { return src_; }

    std::weak_ptr<Node> getDst() { return dst_; }

    void clearSrcDst() {
      src_.reset();
      dst_.reset();
    }

    double getWeight() const { return weight_; }
    void setWeight(double weight) { weight_ = weight; }
  };

  void addNode(const std::string &name) {
    nodes_.emplace_back(std::make_shared<Node>(id, name));
    idToNode_.insert({id, nodes_.back()});
    nameToNode_.insert({name, nodes_.back()});

    ++id;
  }

  void addEdge(const std::string &src, const std::string &dst, double weight);

  std::vector<std::weak_ptr<Node>> getNodes() {
    std::vector<std::weak_ptr<Node>> result;

    for (const auto &node : nodes_) {
      result.emplace_back(node);
    }

    return result;
  }

private:
  size_t id = 0;
  std::vector<std::shared_ptr<Node>> nodes_;
  std::vector<std::shared_ptr<Edge>> edges_;
  std::unordered_map<size_t, std::weak_ptr<Node>> idToNode_;
  std::unordered_map<std::string, std::weak_ptr<Node>> nameToNode_;
};

void Graph::addEdge(const std::string &src, const std::string &dst,
                    double weight) {
  std::shared_ptr<Node> srcNode = nullptr;
  std::shared_ptr<Node> dstNode = nullptr;
  try {
    srcNode = nameToNode_.at(src).lock();
    dstNode = nameToNode_.at(dst).lock();
  } catch (const std::exception &exc) {
    std::cout << "Trying to connect unexisting nodes!\n"
              << '\t' << "src: " << src << '\n'
              << '\t' << "dst: " << dst << '\n';
    return;
  }

  if (!srcNode || !dstNode) {
    std::cout << "Couldn't get some of the nodes!\n"
              << '\t' << "src: " << src << '\n'
              << '\t' << "dst: " << dst << '\n';
    return;
  }

  edges_.emplace_back(std::make_shared<Edge>(srcNode, dstNode, weight));
  srcNode->addOutEdge(edges_.back());
  dstNode->addInEdge(edges_.back());
}

void Graph::emitDot() const {
  std::cout << "Emitting graphviz dot description:\n";

  std::cout << "digraph G {\n";

  for (const auto &edge : edges_) {
    auto src = edge->getSrc().lock();
    auto dst = edge->getDst().lock();
    if (!src || !dst) {
      std::cout << "Dangling edge found!\n";
      continue;
    }

    std::cout << '\t' << src->getName() << " -> " << dst->getName()
              << " [ label = " << edge->getWeight() << " ]\n";
  }

  std::cout << "}\n";
}

void buildGraph(Graph &g) {
  std::vector<std::string> names = {"Gleb", "Vasya", "Vanya", "Djan"};
  std::unordered_map<std::string, double> nameToPaid;
  nameToPaid.insert({"Gleb", 3000});
  nameToPaid.insert({"Vasya", 2000});
  nameToPaid.insert({"Vanya", 1000});

  for (const auto &name : names) {
    g.addNode(name);
  }

  for (const auto &[name_paid, paid] : nameToPaid) {
    for (const auto &name : names) {
      g.addEdge(name, name_paid, paid / names.size());
    }
  }
}

void findCycles(Graph &g) {
  for (auto &node : g.getNodes()) {
    auto gNode = node.lock();
    if (!gNode) {
      continue;
    }

    std::stack<std::shared_ptr<Graph::Node>> dsts;
    auto outEdges = gNode->getOuts();
    for (auto &outEdge : outEdges) {
      auto gOutEdge = outEdge.lock();
      if (!gOutEdge) {
        continue;
      }

      auto dstNode = gOutEdge->getDst().lock();
      if (!dstNode) {
        continue;
      }

      for (auto &dstOutEdge : dstNode->getOuts()) {
        auto gDstOutEdge = dstOutEdge.lock();
        if (!gOutEdge) {
          continue;
        }

        auto dstOutNode = gDstOutEdge->getDst().lock();
        if (!dstNode) {
          continue;
        }

        if (dstOutNode == gNode) {
          std::cout << "Found a cycle!\n"
                    << '\t' << gNode->getName() << " -- " << dstNode->getName()
                    << " -- " << dstOutNode->getName() << '\n';
          auto outWeight = gOutEdge->getWeight();
          auto dstOutWeight = gDstOutEdge->getWeight();
          if (outWeight > dstOutWeight) {
            gOutEdge->setWeight(outWeight - dstOutWeight);
            gDstOutEdge->setWeight(0);
          } else {
            gDstOutEdge->setWeight(dstOutWeight - outWeight);
            gOutEdge->setWeight(0);
          }
          break;
        }
      }
    }

    while (!dsts.empty()) {
      auto currentNode = dsts.top();
      dsts.pop();

      for (auto &outEdge : currentNode->getOuts()) {
      }
    }
  }
}

int main() {
  Graph g;
  buildGraph(g);
  g.emitDot();
  /*
  std::unordered_map<size_t, std::string> idToName;
  std::unordered_map<std::string, size_t> nameToId;

  size_t i{0};


  std::cout << "id to name:\n";
  for (const auto &[id, name] : idToName) {
    std::cout << '\t' << id << ": " << name << '\n';
  }

  std::cout << "name to id:\n";
  for (const auto &[name, id] : nameToId) {
    std::cout << '\t' << name << ": " << id << '\n';
  }

  std::cout << "id to paid:\n";
  for (const auto &[id, paid] : idToPaid) {
    std::cout << '\t' << id << ": " << paid << '\n';
  }

  std::vector<std::vector<double>> am(i, std::vector<double>(i, 0));
  for (size_t id1 = 0; id1 < i; ++id1) {
    for (size_t id2 = 0; id2 < i; ++id2) {
      try {
        am.at(id2).at(id1) = idToPaid.at(id1) / i;
      } catch (const std::exception &exc) {
        am.at(id2).at(id1) = 0;
      }
    }
  }

  std::cout << "i=" << i << '\n';
  for (const auto &v : am) {
    for (const auto &d : v) {
      std::cout << d << ' ';
    }
    std::cout << '\n';
  }

  std::cout << "print graphviz:\n";
  size_t indent = 0;
  std::cout << "digraph {\n";
  ++indent;
  for (size_t id1 = 0; id1 < i; ++id1) {
    for (size_t id2 = 0; id2 < i; ++id2) {
      if (fabs(am.at(id2).at(id1)) < 0.001) {
        continue;
      }
      for (size_t in = 0; in < indent; ++in) {
        std::cout << ' ';
      }
      std::cout << idToName.at(id2) << "->" << idToName.at(id1)
                << " [label=" << am.at(id2).at(id1) << "]\n";
    }
  }
  --indent;
  std::cout << "}\n";

  std::cout << "simplyfying graph:\n";
  for (size_t id1 = 0; id1 < i; ++id1) {
    for (size_t id2 = 0; id2 < i; ++id2) {
      if (id1 != id2) {
        continue;
      }

      if (fabs(am.at(id2).at(id1)) > 0) {
        std::cout << "discarding path " << idToName.at(id2) << " -> "
                  << idToName.at(id1) << " (" << am.at(id2).at(id1) << ")\n";
        am.at(id2).at(id1) = 0;
      }
    }
  }

  std::cout << "print graphviz:\n";
  indent = 0;
  std::cout << "digraph {\n";
  ++indent;
  for (size_t id1 = 0; id1 < i; ++id1) {
    for (size_t id2 = 0; id2 < i; ++id2) {
      if (fabs(am.at(id2).at(id1)) < 0.001) {
        continue;
      }
      for (size_t in = 0; in < indent; ++in) {
        std::cout << ' ';
      }
      std::cout << idToName.at(id2) << "->" << idToName.at(id1)
                << " [label=" << am.at(id2).at(id1) << "]\n";
    }
  }
  --indent;
  std::cout << "}\n";
  */
}
