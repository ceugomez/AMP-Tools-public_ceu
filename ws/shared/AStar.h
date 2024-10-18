#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"



class AStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
};