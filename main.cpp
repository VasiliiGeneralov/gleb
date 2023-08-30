#include <deque>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

using ID = int64_t;

class Graph {
  using Matrix = std::vector<std::vector<double>>;
  ID numberOfVertice_;
  Matrix matrix_;

  std::unordered_map<ID, std::string> idToName_;
  std::unordered_map<ID, double> idToPaid_;

  std::vector<std::vector<ID>> cycles_;

  // methods below together with findCycles() implement an algorithm proposed by
  // Donald B. Johnson in
  // https://www.cs.tufts.edu/comp/150GA/homeworks/hw1/Johnson%2075.PDF
  // DOI: https://doi.org/10.1137/0204007
  void unblockVertex(ID vertexID, std::vector<bool> &blockedVertice,
                     Matrix &blockedEdges);
  bool findCircuit(ID vertexID, ID originID, std::deque<ID> &path,
                   std::vector<bool> &blockedVertice, Matrix &blockedEdges);

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

  void deleteLoopEdges();

  void emitDot() const;

  void findCycles();
};

void Graph::createQualShareEdges() {
  for (ID from = 0; from < numberOfVertice_; ++from) {
    for (ID to = 0; to < numberOfVertice_; ++to) {
      double equalShare = idToPaid_.at(to) / numberOfVertice_;

      addEdge(from, to, equalShare);
    }
  }
}

void Graph::deleteLoopEdges() {
  for (ID srcID = 0; srcID < numberOfVertice_; ++srcID) {
    for (ID dstID = 0; dstID < numberOfVertice_; ++dstID) {
      if (srcID == dstID) {
        double &weight = matrix_.at(srcID).at(dstID);
        idToPaid_.at(srcID) -= weight;
        weight = 0;
      }
    }
  }
}

void Graph::emitDot() const {
  std::cout << "digraph G {\n";
  for (ID id1 = 0; id1 < numberOfVertice_; ++id1) {
    for (ID id2 = 0; id2 < numberOfVertice_; ++id2) {
      double weight = matrix_.at(id1).at(id2);
      if (weight < 0.001) {
        continue;
      }
      std::cout << '\t' << idToName_.at(id1) << "->" << idToName_.at(id2)
                << " [label=" << weight << "]\n";
    }
  }
  std::cout << "}\n";
}

void Graph::unblockVertex(ID vertexID, std::vector<bool> &blockedVertice,
                          Matrix &blockedEdges) {
  blockedVertice.at(vertexID) = false;
  for (ID dstID = 0; dstID < numberOfVertice_; ++dstID) {
    double &weight = blockedEdges.at(vertexID).at(dstID);
    if (weight < 0) {
      continue;
    }
    weight = -1;
    if (blockedVertice.at(dstID)) {
      unblockVertex(dstID, blockedVertice, blockedEdges);
    }
  }
}

bool Graph::findCircuit(ID vertexID, ID originID, std::deque<ID> &path,
                        std::vector<bool> &blockedVertice,
                        Matrix &blockedEdges) {
  bool foundCircuit = false;
  path.push_front(vertexID);
  blockedVertice.at(vertexID) = true;

  for (ID dstID = 0; dstID < numberOfVertice_; ++dstID) {
    double weight = matrix_.at(vertexID).at(dstID);
    if (weight < 0.001) {
      continue;
    }

    if (dstID == originID) {
      std::vector<ID> tmp;
      for (auto it = path.crbegin(), end = path.crend(); it != end; ++it) {
        tmp.push_back(*it);
      }
      tmp.push_back(originID);
      cycles_.emplace_back(std::move(tmp));
      foundCircuit = true;
    } else if (!blockedVertice.at(dstID)) {
      if (findCircuit(dstID, originID, path, blockedVertice, blockedEdges)) {
        foundCircuit = true;
      }
    }
  }

  if (foundCircuit) {
    unblockVertex(vertexID, blockedVertice, blockedEdges);
  } else {
    for (ID dstID = 0; dstID < numberOfVertice_; ++dstID) {
      double weight = matrix_.at(vertexID).at(dstID);
      if (weight < 0.001) {
        continue;
      }

      double &blocked = blockedEdges.at(dstID).at(vertexID);
      if (blocked < 0) {
        blocked = 1;
      }
    }
  }

  path.pop_front();

  return foundCircuit;
}

void Graph::findCycles() {
  std::vector<bool> blockedVertice(numberOfVertice_, false);
  Matrix blockedEdges(numberOfVertice_,
                      std::vector<double>(numberOfVertice_, -1));
  ID originID = 0;
  while (originID < numberOfVertice_) {
    std::deque<ID> path = {originID};
    for (ID dstID = 0; dstID < numberOfVertice_; ++dstID) {
      double weight = matrix_.at(originID).at(dstID);
      if (weight < 0.001) {
        continue;
      }

      blockedVertice.at(dstID) = false;
      blockedEdges.at(dstID) = std::vector<double>(numberOfVertice_, -1);

      findCircuit(dstID, originID, path, blockedVertice, blockedEdges);
    }
    ++originID;
  }

  std::cout << "cycles:\n";
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

  std::cout << "id, name, paid:\n";
  for (size_t j = 0; j < i; ++j) {
    std::cout << '\t' << "id=" << j << ' ' << idToName.at(j) << " paid "
              << idToPaid.at(j) << '\n';
  }

  Graph g(std::move(idToName), std::move(idToPaid));
  g.createQualShareEdges();
  g.emitDot();
  g.deleteLoopEdges();
  g.emitDot();
  g.findCycles();
}
