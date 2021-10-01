<template>
  <div class="cam-conatiner">
    <div class="menu-name">Cam</div>
    <img class="cam">
  </div>
</template>

<style lang="scss" scoped>
  @import url('./HomeBotCam.scss');
</style>

<script>
export default {
  name: 'HomeBotCam',
  state() {
    return {
      cnt: 0,  
    }
  },
  mounted() {
    const img = document.querySelector('.cam')
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
  }
}
</script>