<template>
  <div class="IoTNewCard-container shadows">
    
    <img src="@/assets/icons/devices/light.svg" alt="light" class="icon">
    
    <div class="edit-group">
      <span class="item">이름</span>
      <div>
        <span>{{ device.name }}</span>
        <img src="@/assets/icons/edit.svg" alt="name edit">
      </div>
    </div>

    <div class="edit-group">
      <span class="item">위치</span>
      <div>
        <span>{{ device.position }}</span>
        <img src="@/assets/icons/edit.svg" alt="name edit">
      </div>
    </div>
  
  </div>
</template>

<script>
export default {
  name: 'IoTNewCard',
  props: {
    device: Object,
  }
}
</script>

<style lang="scss" scoped>
  @import './IoTNewCard.scss';
</style>