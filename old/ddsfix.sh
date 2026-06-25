#!/usr/bin/env bash
export CYCLONEDDS_URI='<CycloneDDS>
  <Domain>
    <General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default" /></Interfaces></General>
    <Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer Address="localhost"/><Peer Address="192.168.123.200"/><Peer Address="192.168.123.161"/></Peers><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery>
  </Domain>
</CycloneDDS>'

