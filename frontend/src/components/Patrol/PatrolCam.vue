<template>
  <div class="patrol-cam-container">
    <div class="minimap" @click="modalOpen()">
      <SimulationMap :clickEnable="false" :size="'150'" v-if="!isModalViewed"/>
    </div>
    <ModalView @close-modal="modalOff()" v-if="isModalViewed">
      <div class="my-modal-content">
        <span class="map-text">맵을 터치해 해당 경로를 감시하세요.</span>
        <SimulationMap :clickEnable="true" :size="'300'" :isPatrol="true"/>
      </div>
    </ModalView>
    <img class="patrol-cam">
  </div>
</template>
<script>
import SimulationMap from '@/components/common/SimulationMap.vue'
import ModalView from '@/components/common/ModalView.vue'
export default {
  name: 'PatrolCam',
  components: {
    SimulationMap,
    ModalView,
  },
  data() {
    return {
      isModalViewed: false,
      isClickedMap: false,
    }
  },
  mounted() {
    const img = document.querySelector('.patrol-cam')
    this.$socket.on('cam-streaming', message => {
      const byteCharacters = atob(message)
      const byteNumbers = new Array(byteCharacters.length);
      for (let i = 0; i < byteCharacters.length; i++) {
          byteNumbers[i] = byteCharacters.charCodeAt(i);
      }
      const byteArray = new Uint8Array(byteNumbers)
      const blob = new Blob([byteArray], {type: 'image/jpeg'})
      img.onload = event => {
        URL.revokeObjectURL(event.target.src)
      }
      img.src = URL.createObjectURL(blob)

    })
  },
  methods: {
    btn_pick_up() {
      let data = 1;
      this.$socket.emit('pickupToServer', data);
    },
    btn_put_down() {
      let data = 2;
      this.$socket.emit('putdownToServer', data);
    },
    modalOpen() {
      this.isModalViewed = true
      this.$socket.emit('ismapopenToServer', 1);
    },
    modalOff() {
      this.isModalViewed = false
      this.$socket.emit('ismapopenToServer', 0);
    },
  }
}
</script>
<style lang="scss" scoped>
  @import './PatrolCam.scss';
</style>