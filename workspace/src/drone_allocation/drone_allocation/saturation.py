class Saturation:
    
    @staticmethod
    def apply(list, min_limit, max_limit):
        for i in range(len(list)):
            list[i] = max(
                min_limit,
                min(max_limit, list[i])
            )
        
        return list
