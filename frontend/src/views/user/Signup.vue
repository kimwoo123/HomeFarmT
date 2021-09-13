<template>
  <div>
    <input type="email" v-model="email" placeholder="email">

    <input type="password" id="password" v-model="password" placeholder="password">
    <input type="passwordConfirmation" id="passwordConfirmation" v-model="passwordConfirmation" placeholder="passwordConfirmation">

    <button @click="requestSignup()">버튼</button>
  </div>
</template>

<script>
export default {
  name: 'Signup',
  data() {
    return {
      email: '',
      password: '',
      passwordConfirmation: '',
      error: {
				email: false,
				password: false,
				passwordConfirmation: false,
			},
    }
  },
  methods:{
    requestSignup() {
      const credentials = {
        'email': this.email,
        'password': this.password,
      }
      this.$store.dispatch('requestSignup', credentials)
        .then(() => {
          this.$store.dispatch('requestSignup', credentials)
          console.log('완료메시지')
        })
        .catch(err => {
          console.error(err)
        })
    },
  },
}
</script>

<style lang="scss" scoped>
@import url('./Signup.scss');
</style>