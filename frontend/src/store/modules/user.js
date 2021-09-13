import axios from 'axios'

const state = () => ({

})

const getters = {

}

const mutations = {

}

const actions = {
    requestLogin({ commit }, credentials) {
        commit
        const url = 'api/v1/login'
        const body = credentials
        return axios.post(url, body)
    },
    requestSignup({ state }, payload) {
        state
        const url = '/api/v1/user'
        const body = payload
        return axios.post(url, body)
    }
}

export default {
    namespaced: true,
    state,
    actions,
    mutations,
    getters,
}