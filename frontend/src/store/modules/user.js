import axios from 'axios'

const state = () => ({

})

const getters = {

}

const mutations = {

}

const actions = {
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