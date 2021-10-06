<template>
  <div class="menus">
    <div class="shadows menu" @click="$router.push({ name: 'IoT' })" @mouseover="activeImg('iot')" @mouseout="originImg('iot')">
      <img src="@/assets/icons/menus/iot.svg" alt="iot" id="iot">
      <span>기기 제어</span>
    </div>
    <div class="shadows menu" @click="$router.push({ name: 'Patrol' })" @mouseover="activeImg('patrol')" @mouseout="originImg('patrol')">
      <img src="@/assets/icons/menus/patrol.svg" alt="patrol" id="patrol">
      <span>감시</span>  
    </div>
    <div class="shadows menu" @click="$router.push({ name: 'Schedule' })" @mouseover="activeImg('schedule')" @mouseout="originImg('schedule')">
      <img src="@/assets/icons/menus/schedule.svg" alt="schedule" id="schedule">
      <span>일정</span>  
    </div>
    <div class="shadows menu" @click="$router.push({ name: 'Control' })" @mouseover="activeImg('control')" @mouseout="originImg('control')"> 
      <img src="@/assets/icons/menus/control.svg" alt="control" id="control">
      <span>터틀봇 조작</span>  
    </div>
  </div>
</template>

<script>
export default {
  name: 'HomeMenus',
  methods: {
    activeImg(data) {
      let target = document.querySelector(`#${data}`)
      target.src = require(`@/assets/icons/menus/${data}_active.svg`)
    },
    originImg(data) {
      let target = document.querySelector(`#${data}`)
      target.src = require(`@/assets/icons/menus/${data}.svg`)
    },
  }
}
</script>

<style lang="scss" scoped>
  @import './HomeMenus.scss';
</style>