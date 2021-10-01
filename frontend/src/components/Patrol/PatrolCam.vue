<template>
  <div class="patrol-cam-container">
    <img class="patrol-cam" src="@/assets/images/patrol.png">
  </div>
</template>
<style lang="scss" scoped>
  @import './PatrolCam.scss';
</style>
<script>
export default {
  name: 'PatrolCam',
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
  }
}
</script>

