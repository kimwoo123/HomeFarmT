<template>
  <div class="container">
    <PatrolCam style="margin-bottom: 4px"/>
    <PatrolController style="margin-bottom: 18px" :data="'adda'"/>
    <PatrolPathSetting/>
  </div>
</template>

<style lang="scss" scoped>
  @import './PatrolSetting.scss';
</style>

<script>
import PatrolCam from '../components/Patrol/PatrolCam.vue'
import PatrolController from '../components/Patrol/PatrolController.vue'
import PatrolPathSetting from '../components/Patrol/PatrolPathSetting.vue'

export default {
  name: 'PatrolSetting',
  components: {
    PatrolCam,
    PatrolController,
    PatrolPathSetting,
  }
}
</script>