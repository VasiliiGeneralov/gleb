#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <memory>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using ID = uint64_t;

using Edge = std::pair<ID, ID>;

// bool operator==(const Edge &lhs, const Edge &rhs) {
// return lhs.first == rhs.first && lhs.second == rhs.second;
//}

struct EdgeHash {
  size_t operator()(const Edge &e) const {
    std::hash<ID> hash;
    return hash(e.first) ^ (hash(e.second) << 1);
  }
};

class Graph {
  using Matrix = std::vector<std::vector<double>>;
  size_t numberOfVertice_;
  Matrix matrix_;

  std::unordered_map<ID, std::string> idToName_;
  std::unordered_map<ID, double> idToPaid_;

  std::vector<std::vector<ID>> cycles_;

public:
  Graph(std::unordered_map<ID, std::string> &&idToName,
        std::unordered_map<ID, double> &&idToPaid) {
    idToName_ = std::move(idToName);
    idToPaid_ = std::move(idToPaid);

    numberOfVertice_ = idToName_.size();

    matrix_ = std::vector<std::vector<double>>(
        numberOfVertice_, std::vector<double>(numberOfVertice_, 0));
  }

  void addEdge(ID from, ID to, double weight) {
    matrix_.at(from).at(to) = weight;
  }

  void createQualShareEdges();
  void findCycles();

  void emitDot() const;
};

void Graph::emitDot() const {
  std::cout << "digraph G {\n";
  for (size_t id1 = 0; id1 < numberOfVertice_; ++id1) {
    for (size_t id2 = 0; id2 < numberOfVertice_; ++id2) {
      double weight = matrix_.at(id1).at(id2);
      if (fabs(weight) < 0.001) {
        continue;
      }
      std::cout << '\t' << idToName_.at(id1) << "->" << idToName_.at(id2)
                << " [label=" << weight << "]\n";
    }
  }
  std::cout << "}\n";
}

void Graph::createQualShareEdges() {
  for (size_t from = 0; from < numberOfVertice_; ++from) {
    for (size_t to = 0; to < numberOfVertice_; ++to) {
      double equalShare = idToPaid_.at(to) / numberOfVertice_;

      addEdge(from, to, equalShare);
    }
  }
}

void Graph::findCycles() {
  // main cycle: traverse all vertice
  for (size_t from = 0; from < numberOfVertice_; ++from) {
    // std::deque<ID> path;
    // path.push_back(from);
    //

    std::cout << "checking " << idToName_.at(from) << '\n';

    std::stack<ID> dfsStack;
    dfsStack.push(from);

    std::vector<bool> visited(numberOfVertice_, false);
    std::unordered_set<Edge, EdgeHash> bannedEdge;

    ID prevVertexID = 0;

    while (!dfsStack.empty()) {
      ID dst = dfsStack.top();
      dfsStack.pop();

      // found a cycle
      if (dst == from) {
        std::cout << "found cycle!\n";
        // remember the path
        // std::vector<ID> tmp;
        // for (const auto &id : path) {
        // tmp.push_back(id);
        //}
        // cycles_.emplace_back(std::move(tmp));

        // path.pop_back();

        // reset visited flag to discover all paths
        // std::for_each(visited.begin(), visited.end(),
        //[](auto &&flag) { flag = false; });
        visited.at(prevVertexID) = false;

        std::cout << "banning edge " << idToName_.at(prevVertexID) << " -> "
                  << idToName_.at(dst) << '\n';
        bannedEdge.insert({prevVertexID, dst});
      }

      if (visited.at(dst)) {
        continue;
      }
      visited.at(dst) = true;

      for (size_t to = 0; to < numberOfVertice_; ++to) {
        if (auto it = bannedEdge.find({dst, to}); it != bannedEdge.end()) {
          continue;
        }

        double weight = matrix_.at(dst).at(to);
        if (weight < 0.001) {
          continue;
        }

        dfsStack.push(to);
      }

      prevVertexID = dst;
    }
  }

  for (const auto cycle : cycles_) {
    std::cout << '\t';
    for (const auto &id : cycle) {
      std::cout << idToName_.at(id) << " -> ";
    }
    std::cout << '\n';
  }
}

int main() {
  ID i = 0;
  std::unordered_map<ID, std::string> idToName;
  std::unordered_map<ID, double> idToPaid;

  idToPaid.insert({i, 3000});
  idToName.insert({i++, "Gleb"});

  idToPaid.insert({i, 2000});
  idToName.insert({i++, "Vasya"});

  idToPaid.insert({i, 1000});
  idToName.insert({i++, "Vanya"});

  idToPaid.insert({i, 0});
  idToName.insert({i++, "Djan"});

  for (size_t j = 0; j < i; ++j) {
    std::cout << '\t' << "id=" << j << ' ' << idToName.at(j) << " paid "
              << idToPaid.at(j) << '\n';
  }

  Graph g(std::move(idToName), std::move(idToPaid));
  g.createQualShareEdges();
  g.emitDot();
  g.findCycles();

  /*

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
