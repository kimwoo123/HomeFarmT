<template>
  <div class="patrol-container">
    <PatrolCam style="margin-bottom: 5px"/>
    <PatrolPath/>
    <PatrolAnormal/>
  </div>
</template>
<style>
  @import './Patrol.scss';
</style>
<script>
import PatrolCam from '../components/Patrol/PatrolCam.vue'
import PatrolPath from '../components/Patrol/PatrolPath.vue'
import PatrolAnormal from '../components/Patrol/PatrolAnormal.vue'

export default {
  name: 'Patrol',
  components: {
    PatrolCam,
    PatrolPath,
    PatrolAnormal,
  }

}
</script>

