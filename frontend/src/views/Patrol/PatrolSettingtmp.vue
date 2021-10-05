<template>
  <div>
    <Navbar :title="'경로 편집'" :left_icon="true" :right_text="''" :left_push="'Patrol'" :right_push="''"/>
    <div v-if="isPathDrawing">
      <PatrolCam style="margin-bottom: 4px"/>
      <div>
        1번 경로
        <button>선택</button>
        
        2번 경로
        <button>선택</button>
        
        
        3번 경로
        <button>선택</button>


      </div>
    </div>
    <div v-else>
      <PatrolCam style="margin-bottom: 4px"/>

    </div>
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import PatrolCam from '@/components/Patrol/PatrolCam.vue'

export default {
  name: 'PatrolSetting',
  components: {
    Navbar,
    PatrolCam,
  },
  data() {
    return {
      isPathDrawing: false
    }
  }
}
</script>

<style lang="scss" scoped>
  @import './PatrolSettingtmp.scss';
</style>