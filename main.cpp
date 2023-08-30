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

using ID = int64_t;

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

  ID findCycleImpl(std::vector<bool> &visited,
                   const std::unordered_set<Edge, EdgeHash> &bannedEdges,
                   std::vector<ID> &path, ID origin, ID curr, ID prev) const;

public:
  Graph(std::unordered_map<ID, std::string> &&idToName,
        std::unordered_map<ID, double> &&idToPaid) {
    idToName_ = std::move(idToName);
    idToPaid_ = std::move(idToPaid);

    numberOfVertice_ = static_cast<ID>(idToName_.size());

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
  for (ID id1 = 0; id1 < numberOfVertice_; ++id1) {
    for (ID id2 = 0; id2 < numberOfVertice_; ++id2) {
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
  for (ID from = 0; from < numberOfVertice_; ++from) {
    for (ID to = 0; to < numberOfVertice_; ++to) {
      double equalShare = idToPaid_.at(to) / numberOfVertice_;

      addEdge(from, to, equalShare);
    }
  }
}

ID Graph::findCycleImpl(std::vector<bool> &visited,
                        const std::unordered_set<Edge, EdgeHash> &bannedEdges,
                        std::vector<ID> &path, ID origin, ID curr,
                        ID prev) const {
  if (visited.at(curr)) {
    return curr;
  }

  visited.at(curr) = true;

  for (ID id = 0; id < numberOfVertice_; ++id) {
    if (auto it = bannedEdges.find({curr, id}); it != bannedEdges.end()) {
      std::cout << "skipping " << idToName_.at(curr) << " -> "
                << idToName_.at(id) << '\n';
      continue;
    }

    if (id == prev) {
      continue;
    }

    double weight = matrix_.at(curr).at(id);
    if (weight < 0.001) {
      continue;
    }

    std::cout << "banned edges:\n";
    for (const auto &e : bannedEdges) {
      std::cout << '\t' << idToName_.at(e.first) << " -> "
                << idToName_.at(e.second) << '\n';
    }

    ID k = findCycleImpl(visited, bannedEdges, path, origin, id, curr);
    if (-1 != k) {
      path.push_back(curr);
      if (k == curr) {
        break;
      }
      return k;
    }
  }

  return -1;
}

void Graph::findCycles() {
  // main cycle: traverse all vertice
  for (size_t from = 0; from < /* numberOfVertice_ */ 1; ++from) {
    std::cout << "checking " << idToName_.at(from) << '\n';
    std::vector<bool> visited(numberOfVertice_, false);
    std::unordered_set<Edge, EdgeHash> bannedEdges;
    std::vector<ID> path;
    path.push_back(from);

    int i = 0;
    do {
      // find a cycle
      path.clear();
      path.push_back(from);
      findCycleImpl(visited, bannedEdges, path, from, from, -1);

      // reset visited flags
      std::for_each(visited.begin(), visited.end(),
                    [](auto &&flag) { flag = false; });

      // remember path
      std::cout << "path:\n";
      for (const auto &v : path) {
        std::cout << v << " -> ";
      }
      std::cout << '\n';

      std::vector<ID> tmp;
      for (auto it = path.crbegin(), end = path.crend(); it != end; ++it) {
        tmp.push_back(*it);
      }
      cycles_.push_back(tmp);

      std::cout << "tmp:\n";
      for (const auto &v : tmp) {
        std::cout << v << " -> ";
      }
      std::cout << '\n';

      // ban edge to find another paths
      ID edgeToBanSrc = -1;
      ID edgeToBanDst = -1;
      std::vector<ID> lastPath = cycles_.back();
      size_t lastPathLen = lastPath.size();
      if (2 == lastPathLen) {
        // 1 -> 1, ban 1 -> 1
        edgeToBanSrc = lastPath.at(0);
        edgeToBanDst = lastPath.at(1);
      } else if (3 == lastPathLen) {
        if (lastPath.back() == lastPath.front()) {
          // 1 -> 2 -> 1, ban 2 -> 1
          edgeToBanSrc = lastPath.at(1);
          edgeToBanDst = lastPath.at(2);
        } else {
          // 1 -> 2 -> 2
          edgeToBanSrc = lastPath.back();
          edgeToBanDst = lastPath.back();
        }
      } else if (lastPathLen > 3) {
        // 1 -> 2 -> 3 -> 1, ban 2 -> 3
        edgeToBanSrc = lastPath.at(lastPathLen - 3);
        edgeToBanDst = lastPath.at(lastPathLen - 2);
      }
      if (-1 != edgeToBanSrc && -1 != edgeToBanDst) {
        std::cout << "banning edge " << idToName_.at(edgeToBanSrc) << " -> "
                  << idToName_.at(edgeToBanDst) << '\n';
      }
      bannedEdges.insert({edgeToBanSrc, edgeToBanDst});
      ++i;
    } while (path.size() > 1 && i < 30);

    std::cout << "banned edges:\n";
    for (const auto &edge : bannedEdges) {
      if (-1 == edge.first || -1 == edge.second) {
        continue;
      }
      std::cout << '\t' << edge.first << " (" << idToName_.at(edge.first) << ")"
                << " -> " << edge.second << " (" << idToName_.at(edge.second)
                << ")" << '\n';
    }
  }

  for (const auto &cycle : cycles_) {
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
