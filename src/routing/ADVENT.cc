//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "ADVENT.h"

Define_Module(ADVENT);

const simsignalwrap_t ADVENT::mobilityStateChangedSignal = simsignalwrap_t(MIXIM_SIGNAL_MOBILITY_CHANGE_NAME);

void ADVENT::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);

    if (stage == 0) {
        traci = Veins::TraCIMobilityAccess().get(getParentModule());

        dataRateTimer = new cMessage("datarate", SEND_DATA);
        datarate = par("datarate").doubleValue();

        //TODO: Added for Game Theory Solution
        lastTxPower = registerSignal("lastTxPower");
        meanSNR = registerSignal("meanSNR");

        sentDownMACInCCH = registerSignal("sentDownMACInCCH");
        collisions = registerSignal("collisions");
        messagesTransmittedSCF = registerSignal("messagesTransmittedSCF");
        messagesReceivedSCF = registerSignal("messagesReceivedSCF");
        duplicatedMessages = registerSignal("duplicatedMessages");
        messagesTransmitted = registerSignal("messagesTransmitted");
        messagesReceived = registerSignal("messagesReceived");
        isInROI = registerSignal("isInROI");

        wasInROI = false;
        disseminationStarted = false;

        simulation.getSystemModule()->subscribe("disseminationStartTime", this);


        mac = FindModule<Mac1609_4*>::findSubModule(getParentModule());
        assert(mac);

        //TODO: Added for Game Theory Solution
        curTxPower = mac->par("txPower");
        powerLevel = 3;
        myUtilityValue = 0;

        lastNumPktLost = 0;
        lastNumCollisions = 0;
        totalCollisions = 0;

        CCHStartTimer = new cMessage("CCH start", CCH_START);
        SCHStartTimer = new cMessage("SCH start", SCH_START);

        uint64_t currenTime = simTime().raw();
        uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
        double timeToNextSwitch = (double)(switchingTime - (currenTime % switchingTime)) / simTime().getScale();

        // Control Channel is active
        if ((currenTime / switchingTime) % 2 == 0) {
            scheduleAt(simTime() + timeToNextSwitch + SWITCHING_INTERVAL_11P, CCHStartTimer);
            scheduleAt(simTime() + timeToNextSwitch, SCHStartTimer);
        }
        // Service Channel is active
        else {
            scheduleAt(simTime() + timeToNextSwitch, CCHStartTimer);
            scheduleAt(simTime() + timeToNextSwitch + SWITCHING_INTERVAL_11P, SCHStartTimer);
        }
    }
}

void ADVENT::finish() {
    BaseWaveApplLayer::finish();

    if (wasInROI) {
        emit(isInROI, 1);
    } else {
        emit(isInROI, 0);
    }
    //TODO: Added for Game Theory Solution
    emit(lastTxPower, curTxPower);

    emit(collisions, totalCollisions);

    if (!messagesRcvd.empty()) {
        std::ofstream log;
        std::ostringstream o;

        o << "./results/" << par("log_traffic").longValue() << "-" << par("log_replication").longValue() << "-receiver-" << myId;
        log.open(o.str().c_str());

        for (std::map<int, MessageInfoEntry*>::iterator i = messagesRcvd.begin(); i != messagesRcvd.end(); i++) {
            MessageInfoEntry* videoInfo = i->second;

            log << videoInfo->receptionTime << " " << "id " << videoInfo->messageID << " " << "udp " << videoInfo->messageLength << " " << videoInfo->distanceToOrigin << endl;
        }
        log.close();
    }
    simulation.getSystemModule()->unsubscribe("disseminationStartTime", this);
}

void ADVENT::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {

        case SEND_BEACON_EVT: {
            WaveShortMessage* wsm = prepareWSM("beacon", beaconLengthBits, type_CCH, beaconPriority, 0, -1);
            BeaconMessage* beaconMsg = new BeaconMessage("beacon");
            beaconMsg->setSenderDirection(traci->getAngleRad());

            beaconMsg->setReceivedMessagesArraySize(messagesRcvd.size());
            unsigned int j = 0;
            for (std::map<int, MessageInfoEntry*>::iterator i = messagesRcvd.begin(); i != messagesRcvd.end(); i++) {
                int messageID = i->first;

                beaconMsg->setReceivedMessages(j, messageID);
                j++;
            }

            wsm->encapsulate(beaconMsg);

            Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());
            if (simTime() > par("startDataProductionTime").doubleValue() - 3 &&
                    curPosition.distance(rsuPosition) <= par("dataROI").doubleValue() + 300) {

                sendWSM(wsm);
            } else {
                delete wsm;
            }

            scheduleAt(simTime() + par("beaconInterval").doubleValue(), sendBeaconEvt);
            break;
        }

        case BROADCAST_TIMEOUT: {
            MessageInfoEntry* info = (MessageInfoEntry*) msg->getContextPointer();

            if (isInsideROI(info) && isMessageAlive(info)) {
                addMessageToOutputQueue(info);
            }

            delete info->broadcastTimer;
            info->broadcastTimer = NULL;

            break;
        }

        case SCF_TIMEOUT: {
            MessageInfoEntry* info = (MessageInfoEntry*) msg->getContextPointer();

            if (isInsideROI(info) && isMessageAlive(info)) {
                addMessageToOutputQueue(info);
            }

            delete info->scfTimer;
            info->scfTimer = NULL;

            break;
        }

        case SEND_DATA: {
            sendData();

            break;
        }

        case BACK_TRAFFIC_ENTRY_TIMEOUT: {
            NeighborEntry* entry = (NeighborEntry*) msg->getContextPointer();
            int address = entry->senderAddress;
            cancelAndDelete(entry->beaconHoldTimer);
            entry->beaconHoldTimer = NULL;
            lastRequesters.erase(address);

            break;
        }

        case CCH_START: {
            totalCollisions = totalCollisions + mac->statsTXRXLostPackets - lastNumCollisions;
            scheduleAt(simTime() + SWITCHING_INTERVAL_11P + SWITCHING_INTERVAL_11P, CCHStartTimer);
            break;
        }

        case SCH_START: {
            lastNumPktLost = mac->statsSNIRLostPackets + mac->statsTXRXLostPackets;
            lastNumCollisions = mac->statsTXRXLostPackets;
            scheduleAt(simTime() + SWITCHING_INTERVAL_11P + SWITCHING_INTERVAL_11P, SCHStartTimer);
            break;
        }

        default: {
            if (msg)
                EV << "ADVENT - Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
            break;
        }
    }
}

void ADVENT::onBeacon(WaveShortMessage* wsm) {
    scfPolicyMissingPackets(wsm);

    Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());
    // if back-traffic is enabled, then generate it only three seconds before the main dissemination.
    if (par("generateBackTraffic").boolValue() && simTime() > par("startDataProductionTime").doubleValue() - 3
            && curPosition.distance(rsuPosition) <= par("dataROI").doubleValue() + 300) {
        processBackTraffic(wsm->getSenderAddress());
    }
}

void ADVENT::onData(WaveShortMessage* wsm) {
    //TODO: Added for GAme Theory Solution
    if (par("adaptTxPower").boolValue())
        adjustTxPower(wsm);

    DataMessage* dataMsg = dynamic_cast<DataMessage*>(wsm->decapsulate());

    MessageInfoEntry* info = new MessageInfoEntry;
    info->messageID = wsm->getSerial();
    info->messageOriginPosition = dataMsg->getMessageOriginPosition();
    info->messageROI = dataMsg->getMessageROI();
    info->messageOriginTime = dataMsg->getMessageOriginTime();
    info->messageTTL = dataMsg->getMessageTTL();
    info->hops = dataMsg->getHops() + 1;
    info->sentSCFState = false;
    info->receptionTime = simTime();
    info->messageLength = wsm->getByteLength();
    info->distanceToOrigin = info->messageOriginPosition.distance(curPosition);
    info->broadcastTimer = NULL;
    info->scfTimer = NULL;

    if (!isInsideROI(info) || !isMessageAlive(info)) {
        delete info;
        delete dataMsg;

        return;
    }

    if (!isDuplicateMsg(info->messageID)) {
        double percentageDistance, delay;

        percentageDistance = std::min(wsm->getSenderPos().distance(curPosition), par("avgCommRange").doubleValue()) / par("avgCommRange").doubleValue();

        if (info->hops == 1 || isInsideForwardingZone(wsm->getSenderPos(), dataMsg->getSenderDirection())) {
            delay = par("maxDelay").doubleValue() * (1.0 - percentageDistance);
        } else {
            delay = par("maxDelay").doubleValue() * (2.0 - percentageDistance);
        }

        info->broadcastTimer = new cMessage("broadcast suppression", BROADCAST_TIMEOUT);
        info->broadcastTimer->setContextPointer((MessageInfoEntry*) info);

        // Adjust delay to CCH and SCH cycles
        delay = desynchronize(delay);
        scheduleAt(simTime() + delay, info->broadcastTimer);

        messagesRcvd[info->messageID] = info;
        emit(messagesReceived, 1);
        if (dataMsg->getSentSCFState()) {
            emit(messagesReceivedSCF, 1);
        }

    } else {
        delete info;
        MessageInfoEntry* info = messagesRcvd[wsm->getSerial()];

        // If node is in broadcast suppression, then leave it
        if ((info->broadcastTimer && info->broadcastTimer->isScheduled())
            || (info->scfTimer && info->scfTimer->isScheduled())
            || (std::find(outputQueue.begin(), outputQueue.end(), info->messageID) != outputQueue.end())) {

            cancelAndDelete(info->broadcastTimer);
            info->broadcastTimer = NULL;

            cancelAndDelete(info->scfTimer);
            info->scfTimer = NULL;

            outputQueue.remove(info->messageID);
        }
        emit(duplicatedMessages, 1);
    }

    delete dataMsg;
}

void ADVENT::addMessageToOutputQueue(MessageInfoEntry* info) {
    // Adjust dataRateTimer to send message down to MAC only when the
    // Service Channel is active. If current time is Control Channel, then
    // schedule time to the next Service Channel cycle
    if (std::find(outputQueue.begin(), outputQueue.end(), info->messageID) == outputQueue.end()) {
        outputQueue.push_back(info->messageID);
        // Schedule to the beginning of next SCH
        if (!dataRateTimer->isScheduled()) {
            uint64_t currenTime = simTime().raw();
            uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
            double timeToNextSwitch = (double)(switchingTime - (currenTime % switchingTime)) / simTime().getScale();
            if ((currenTime / switchingTime) % 2 == 0) {
                scheduleAt(simTime() + timeToNextSwitch + 0.000000000001, dataRateTimer);
            }
            else {
                scheduleAt(simTime(), dataRateTimer);
            }
        }
    }
}

void ADVENT::sendData() {
    // Adjust dataRateTimer to send message down to MAC only when the
    // Service Channel is active. If current time is Control Channel, then
    // schedule time to the next Service Channel cycle
    if (!outputQueue.empty()) {
        int nextMessageID = outputQueue.front();
        outputQueue.pop_front();
        MessageInfoEntry* info = messagesRcvd[nextMessageID];

        simtime_t time;

        if (isInsideROI(info) && isMessageAlive(info)) {

            WaveShortMessage* wsm = createDataMsg(info);
            sendWSM(wsm);

            if (info->sentSCFState) {
                emit(messagesTransmittedSCF, 1);
            }
            emit(messagesTransmitted, 1);

            if (isCCHActive()) {
                emit(sentDownMACInCCH, 1);
            }

            long lostPkts = (mac->statsSNIRLostPackets + mac->statsTXRXLostPackets) - lastNumPktLost;

            double nextPktTime = wsm->getBitLength() / (datarate / (1 + lostPkts));
            time = simTime() + nextPktTime + individualOffset;
        } else {
            time = simTime();
        }

        // Consider the individual offset of each node
        uint64_t currenTime = time.raw();
        uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
        double timeToNextSwitch = (double)(switchingTime - (currenTime % switchingTime)) / simTime().getScale();
        // Control Channel is active
        if ((currenTime / switchingTime) % 2 == 0) {
            scheduleAt(time + timeToNextSwitch + 0.000000000001, dataRateTimer);
        // Service Channel is active
        } else {
            scheduleAt(time, dataRateTimer);
        }
    }
}

double ADVENT::desynchronize(double delay) {
    // Adjust dataRateTimer to send message down to MAC only when the
    // Service Channel is active. If current time is Control Channel, then
    // schedule time to the next Service Channel cycle
    uint64_t currenTime = simTime().raw();
    uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();
    double timeToNextSwitch = (double)(switchingTime - (currenTime % switchingTime)) / simTime().getScale();
    // Control Channel is active
    if ((currenTime / switchingTime) % 2 == 0) {
        int rounds = floor(delay / 0.05);
        delay = timeToNextSwitch + delay + (rounds * 0.05) + 0.000000000001;

    // Service Channel is active
    } else {
        double delay_tmp = delay - timeToNextSwitch;
        if (delay_tmp > 0) {
            int rounds = ceil(delay_tmp / 0.05);
            delay = delay + (rounds * 0.05);
        }
    }

    return delay;
}

bool ADVENT::isCCHActive() {
    uint64_t currenTime = simTime().raw();
    uint64_t switchingTime = SWITCHING_INTERVAL_11P.raw();

    return ((currenTime / switchingTime) % 2 == 0);
}

//TODO: Added for Game Theory Solution
void ADVENT::adjustTxPower(WaveShortMessage* wsm) {
    double rcvTxPower, rcvSNR, utility;

    rcvTxPower = ((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getRecvPower_dBm();
    // Convert dBm to mW
    rcvTxPower = pow(10.0, rcvTxPower / 10.0);
    rcvSNR = ((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getSnr();

    utility = (1 * log(1 + rcvSNR) - (0.009 * curTxPower));

    //if (rcvTxPower >= curTxPower) {
        if (myUtilityValue > utility) {
            myUtilityValue = utility;
            //increaseTxPower();
        } else {
            decreaseTxPower();
        }
    //}

    emit(meanSNR, rcvSNR);
}

//TODO: Added for Game Theory Solution
void ADVENT::decreaseTxPower() {
    const double txPowerVals[] = {0.61, 0.98, 1.6, 2.2};

    if (powerLevel > 0) {
        powerLevel--;
        curTxPower = txPowerVals[powerLevel];
    }
}

//TODO: Added for Game Theory Solution
void ADVENT::increaseTxPower() {
    const double txPowerVals[] = {0.61, 0.98, 1.6, 2.2};

    if (powerLevel < 3) {
        powerLevel++;
        curTxPower = txPowerVals[powerLevel];
    }

}

bool ADVENT::isInsideForwardingZone(Coord senderPos, double senderDirection) {
    double fwdZoneSize = par("forwardingZoneSize").doubleValue();
    double tan = atan2(curPosition.y - senderPos.y, curPosition.x - senderPos.x);
    double angle = fabs(tan - senderDirection);

    return (angle >= 0 && angle <= fwdZoneSize) // 0 <= angle <= theta
            || (angle >= ((M_PI / 2) - fwdZoneSize) && angle <= ((M_PI / 2) + fwdZoneSize)) // 90 - theta <= angle <= 90 + theta
            || (angle >= (M_PI - fwdZoneSize) && angle <= (M_PI + fwdZoneSize)) // 180 - theta <= angle <= 180 + theta
            || (angle >= ((3 * M_PI / 2) - fwdZoneSize) && angle <= ((3 * M_PI / 2) + fwdZoneSize)) // 270 - theta <= angle <= 270 + theta
            || (angle >= ((2 * M_PI) - fwdZoneSize) && angle <= (2 * M_PI)); // 360 - theta <= angle <= 360
}

void ADVENT::scfPolicyMissingPackets(WaveShortMessage* wsm) {
    BeaconMessage* beaconMsg = dynamic_cast<BeaconMessage*>(wsm->decapsulate());

    simtime_t time = simTime();

    for (std::map<int, MessageInfoEntry*>::iterator i = messagesRcvd.begin(); i != messagesRcvd.end(); i++) {
        bool received = false;
        MessageInfoEntry* info = i->second;
        if (info->scfTimer == NULL) {
            int numMsgsRcvd = beaconMsg->getReceivedMessagesArraySize();

            for (int j = 0; j < numMsgsRcvd; j++) {
                int msgRcvdID = beaconMsg->getReceivedMessages(j);
                if (msgRcvdID == info->messageID) {
                    received = true;
                    break;
                }
            }
            if (!received) {
                info->sentSCFState = true;

                double percentageDistance, delay;

                percentageDistance = std::min(wsm->getSenderPos().distance(curPosition), par("avgCommRange").doubleValue()) / par("avgCommRange").doubleValue();

                delay = (par("maxDelay").doubleValue() * 4) * percentageDistance;

                info->scfTimer = new cMessage("scf suppression", SCF_TIMEOUT);
                info->scfTimer->setContextPointer((MessageInfoEntry*) info);

                // Adjust delay to CCH and SCH cycles
                delay = desynchronize(delay);

                scheduleAt(time + delay, info->scfTimer);
                time = time + ((info->messageLength * 8) / datarate);
            }
            received = false;
        }
    }

    delete beaconMsg;
}

bool ADVENT::isDuplicateMsg(int messageID) {
    return messagesRcvd.find(messageID) != messagesRcvd.end();
}

bool ADVENT::isInsideROI(MessageInfoEntry* info) {
    return info->messageOriginPosition.distance(curPosition) < info->messageROI;
}

bool ADVENT::isMessageAlive(MessageInfoEntry* info) {
    return simTime() < info->messageOriginTime + info->messageTTL;
}

WaveShortMessage* ADVENT::createDataMsg(MessageInfoEntry* info) {
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, type_SCH, dataPriority, 0, info->messageID);

    //TODO: Added for Game Theory Solution
    PhyControlMessage *controlInfo = new PhyControlMessage();
    controlInfo->setTxPower_mW(curTxPower);
    wsm->setControlInfo(dynamic_cast<cObject *>(controlInfo));

    DataMessage* msg = new DataMessage("data");

    msg->setSenderDirection(traci->getAngleRad());

    msg->setMessageOriginPosition(info->messageOriginPosition);
    msg->setMessageROI(info->messageROI);
    msg->setMessageOriginTime(info->messageOriginTime);
    msg->setMessageTTL(info->messageTTL);
    msg->setHops(info->hops);

    if (info->sentSCFState) {
        msg->setSentSCFState(true);
    }

    wsm->setByteLength(info->messageLength);

    wsm->encapsulate(msg);

    return wsm;
}

void ADVENT::processBackTraffic(int senderAddr) {
    if (lastRequesters.find(senderAddr) == lastRequesters.end()) {
        // Send 60 packets of 1000 bytes on the Service Channel
        for (int i = 0; i < 60; i++) {
            WaveShortMessage* wsm = prepareWSM("back traffic", dataLengthBits, type_SCH, dataPriority, 0, i);
            wsm->setByteLength(1000);

            sendWSM(wsm);
        }

        NeighborEntry* entry = new NeighborEntry;

        entry->beaconHoldTimer = new cMessage("beacon entry timeout", BACK_TRAFFIC_ENTRY_TIMEOUT);
        entry->beaconHoldTimer->setContextPointer((NeighborEntry*) entry);
        scheduleAt(simTime() + 3, entry->beaconHoldTimer);

        lastRequesters[senderAddr] = entry;
    }
}

void ADVENT::addGPSError() {
    cRNG *rng = getRNG(9);
    double randomNumber, error;

    randomNumber = rng->doubleRand(); // Generate value in [0, 1)
    error = randomNumber * par("maxGPSError").doubleValue();

    if (randomNumber > 0.5) {
        curPosition.x += error;
        curPosition.y += error;
    } else {
        curPosition.x -= error;
        curPosition.y -= error;
    }
}

void ADVENT::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    BaseWaveApplLayer::receiveSignal(source, signalID, obj, details);

    if (signalID == mobilityStateChangedSignal) {
        Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());

        if (disseminationStarted && curPosition.distance(rsuPosition) <= par("dataROI").doubleValue() &&
                ((simTime() >= disseminationStartTime) && (simTime() <= (disseminationStartTime + par("dataTTL").doubleValue())))) {
            wasInROI = true;
        }

        if (par("maxGPSError").doubleValue() > 0) {
            addGPSError();
        }
    }
}

void ADVENT::receiveSignal(cComponent *source, simsignal_t signalID, const SimTime& t) {
    Enter_Method_Silent();

    if (!strcmp(getSignalName(signalID), "disseminationStartTime")) {
        disseminationStarted = true;
        disseminationStartTime = simTime();
        Coord rsuPosition = Coord(par("eventOriginX").doubleValue(), par("eventOriginY").doubleValue(), par("eventOriginZ").doubleValue());

        if (curPosition.distance(rsuPosition) <= par("dataROI").doubleValue())
            wasInROI = true;
    }
}

ADVENT::~ADVENT() {
    for (std::map<int, MessageInfoEntry*>::iterator i = messagesRcvd.begin(); i != messagesRcvd.end(); i++) {
        MessageInfoEntry* info = i->second;

        cancelAndDelete(info->broadcastTimer);
        cancelAndDelete(info->scfTimer);
    }
    for (std::map<int, NeighborEntry*>::iterator i = lastRequesters.begin(); i != lastRequesters.end(); i++) {
        NeighborEntry* entry = i->second;
        cancelAndDelete(entry->beaconHoldTimer);
    }
    cancelAndDelete(dataRateTimer);
    cancelAndDelete(CCHStartTimer);
    cancelAndDelete(SCHStartTimer);
}
