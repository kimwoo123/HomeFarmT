module.exports = {
  css: {
    loaderOptions: {
      sass: {
        additionalData: `
          @import "@/assets/scss/normalize.scss";
          @import "@/assets/scss/common.scss";
        `
      }
    }
  }
}