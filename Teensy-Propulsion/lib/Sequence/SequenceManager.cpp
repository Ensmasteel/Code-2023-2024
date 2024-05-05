#include "SequenceManager.h"

SequenceManager::SequenceManager(std::vector<Sequence> sequences, unsigned int curSeqId):
    sequences(sequences),
    curSeqId(curSeqId)
{}

void SequenceManager::setEnemy(bool enemy, float enemyDst, float enemyAng){
    enemy = enemy;
    enemyDst = enemyDst;
    enemyAng = enemyAng;
}

void SequenceManager::forceRetourBase() {
    curSeqId = sequences.size() - 1;
}

void SequenceManager::update(float dt, Robot* robot){
    if (curSeqId < sequences.size ()) { // there is still a sequence to do
        if (enemy) {
            if (sequences[curSeqId].checkClearPath(enemyDst, enemyAng)){
                // the enemy is not on our path

                enemy = false;
                enemyDst = -1.0f;
                enemyAng = 0.0f;
                
                sequences[curSeqId].run(dt, robot);
            } else {
                // we cannot pursue our sequence as there is someone on the way

                // TODO: what can we do?

                robot->stopMovement();
            }
        } else {
            sequences[curSeqId].run(dt, robot);
        }

        if (sequences[curSeqId].isDone()) {
            sequences[curSeqId].reset();    // reset the sequences to be able to reuse it later
            curSeqId++;
        }
    }
}
