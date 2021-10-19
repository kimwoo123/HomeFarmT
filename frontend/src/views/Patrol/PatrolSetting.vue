<template>
  <div>
    <Navbar :title="'경로 편집'" :left_icon="true" :right_text="''" :left_push="'Patrol'" :right_push="''"/>
    <div v-if="!isPathDrawing">
      <PatrolCam style="margin-bottom: 4px"/>
      <div class="paths">

        <div class="path shadows" @click="change_path(1)" :class="{'path-active' : currentPath==='1'}">
          <div class="title" v-if="editNum && editNum==1" @focusout="edit(1)" @keypress.enter="edit(1)">
            <input type="text" v-model="path1" class="input-title">
          </div>
          <div class="title" v-else @click="editNum=1">
            <span>{{ path1 }}</span>
            <img src="@/assets/icons/edit.svg" alt="edit">
          </div>
          <span class="path-re-btn" @click="btn_new_path_on(1)">경로 재설정</span>
        </div>

        <div class="path shadows" @click="change_path(2)" :class="{'path-active' : currentPath==='2'}">
          <div class="title" v-if="editNum && editNum==2" @focusout="edit(2)" @keypress.enter="edit(2)">
            <input type="text" v-model="path2" class="input-title">
          </div>
          <div class="title" v-else @click="editNum=2">
            <span>{{ path2 }}</span>
            <img src="@/assets/icons/edit.svg" alt="edit">
          </div>
          <span class="path-re-btn" @click="btn_new_path_on(2)">경로 재설정</span>
        </div>

        <div class="path shadows" @click="change_path(3)" :class="{'path-active' : currentPath==='3'}">
          <div class="title" v-if="editNum && editNum==3" @focusout="edit(3)" @keypress.enter="edit(3)">
            <input type="text" v-model="path3" class="input-title">
          </div>
          <div class="title" v-else @click="editNum=3">
            <span>{{ path3 }}</span>
            <img src="@/assets/icons/edit.svg" alt="edit">
          </div>
          <span class="path-re-btn" @click="btn_new_path_on(3)">경로 재설정</span>
        </div>

      </div>
    </div>
    <div v-else>
      <div class="minimap" @click="modalOpen()">
        <SimulationMap :clickEnable="false" :size="'150'" v-if="!isModalViewed"/>
      </div>
      <ModalView @close-modal="modalOff()" v-if="isModalViewed">
        <div class="my-modal-content">
          <span class="map-text">맵을 터치해 해당 경로를 감시하세요.</span>
          <SimulationMap :clickEnable="true" :size="'300'" :isPatrol="true"/>
        </div>
      </ModalView>
      <ControlCam style="margin-bottom: 30px;"/>
      <button class="my-btn" @click="btn_new_path_off()">경로 재설정 완료</button>
      <Controller style="margin-top: -230px;" />
    </div>
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import PatrolCam from '@/components/Patrol/PatrolCam.vue'
import Controller from '@/components/common/Controller.vue'
import ControlCam from '@/components/Control/ControlCam.vue'
import SimulationMap from '@/components/common/SimulationMap.vue'
import ModalView from '@/components/common/ModalView.vue'

export default {
  name: 'PatrolSetting',
  components: {
    Navbar,
    PatrolCam,
    Controller,
    ControlCam,
    SimulationMap,
    ModalView,
  },
  data() {
    return {
      isPathDrawing: false,
      isModalViewed: false,
      isClickedMap: false,
      currentPath: localStorage.getItem('currentPath') || '1',
      path1: localStorage.getItem('path1') || '1번 경로',
      path2: localStorage.getItem('path2') || '2번 경로',
      path3: localStorage.getItem('path3') || '3번 경로',
      editNum: '',
    }
  },
  methods: {
    change_path(data) {
      localStorage.setItem('currentPath', data)
      this.currentPath = String(data)
      this.$socket.emit('changePathToServer', data);
    },
    btn_new_path_on(data) {
      this.isPathDrawing = true
      this.$socket.emit('newPathOnToServer', data);
    },
    btn_new_path_off() {
      this.isPathDrawing = false
      this.$socket.emit('newPathOffToServer');
    },
    edit(data) {
      this.editNum = 0
      switch(data) {
        case 1:
          console.log(this.path1)
          localStorage.setItem('path1', this.path1);
          break;
        case 2:
          localStorage.setItem('path2', this.path2);
          break;
        case 3:
          localStorage.setItem('path3', this.path3);
          break;
      }
    }
  }
}
</script>

<style lang="scss" scoped>
  @import './PatrolSetting.scss';
</style>